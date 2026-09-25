// ============================================================================
// server.js - Backend Web cho hệ thống AGV
// ============================================================================
const express = require('express');
const cors = require('cors');
const http = require('http');
const WebSocket = require('ws');
const path = require('path');
const db = require('./db');

const AGV_WS_URL = process.env.AGV_WS_URL || 'ws://localhost:8080';
const PORT = process.env.PORT || 3000;

// Đường dẫn C++ Core dùng để lưu bản đồ (data/map_layout.json, xem AGVserver.cpp
// -> MAP_SAVE_PATH trong main()). Khai báo lại ở đây để: (1) Node biết chính xác
// nơi C++ Core sẽ ghi file khi không truyền "path" tuỳ ý, và (2) hiển thị đúng
// đường dẫn đó cho người dùng trên dashboard khi bấm "Lưu nhanh".
const CORE_MAP_SAVE_PATH = 'data/map_layout.json';

// Nhận diện các dòng nhật ký từ C++ Core mô tả "sự cố xe" (lỗi / mất ACK lệnh /
// mất kết nối) để đếm vào KPI "Sự cố xe". Các chuỗi này do FleetManager phát ra
// (xem FleetManager.hpp -> emit(...)), giữ nguyên văn nên khớp được bằng regex.
const INCIDENT_RE = /^AGV_(\S+)\s+(BAO LOI|KHONG ACK lenh|MAT KET NOI)/;

const app = express();
app.use(cors());
app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

const server = http.createServer(app);

let latestState = {
    map: { nodes: [], edges: [], obstacles: [] },
    fleet: { agvs: [], pending_tasks: [] },
    mutex: {},
    logs: []
};

let agvConnected = false;
const previousAgvTargets = new Map();
let agvSocket = null;

function connectToAgvServer() {
    console.log(`[AGV-LINK] Dang ket noi toi ${AGV_WS_URL} ...`);
    agvSocket = new WebSocket(AGV_WS_URL);

    agvSocket.on('open', () => {
        agvConnected = true;
        console.log('[AGV-LINK] Da ket noi AGVServer.cpp thanh cong.');
        broadcastToBrowsers({ ...latestState, agv_connected: true });

        const savedMap = db.getLatestMapSnapshot();
        if (savedMap && savedMap.nodes && savedMap.nodes.length > 0) {
            latestState.map = savedMap;
            sendCommandToAgv({ cmd: 'load_map_data', map: savedMap });
        }

        // GHI CHÚ: KHÔNG còn tự seed AGV_01 mặc định ở đây. Theo quy tắc mới, AGV chỉ
        // xuất hiện khi ESP32 thật của xe đó kết nối và gửi telemetry đầu tiên qua MQTT
        // tới C++ Core (xem AGVserver.cpp -> autoRegisterAgvIfNeeded()), lúc đó xe được
        // gán lần lượt vào trạm sạc 1, 2, 3... theo đúng thứ tự kết nối.
    });

    agvSocket.on('message', raw => {
        try {
            onAgvState(JSON.parse(raw.toString()));
        } catch (error) {
            console.error('[AGV-LINK] Loi parse JSON:', error.message);
        }
    });

    agvSocket.on('close', () => {
        const wasConnected = agvConnected;
        agvConnected = false;
        console.warn('[AGV-LINK] Mat ket noi AGVServer.cpp.');

        if (wasConnected) {
            try {
                const report = db.computeKpiReport('disconnect');
                console.log('[KPI] Da tao bao cao KPI:', report);
                broadcastToBrowsers({ ...latestState, agv_connected: false, kpi_alert: report });
            } catch (error) {
                console.error('[KPI] Loi:', error.message);
                broadcastToBrowsers({ ...latestState, agv_connected: false });
            }
        } else {
            broadcastToBrowsers({ ...latestState, agv_connected: false });
        }
        setTimeout(connectToAgvServer, 3000);
    });

    agvSocket.on('error', error => {
        console.error('[AGV-LINK] Loi ket noi:', error.message);
    });
}

function sendCommandToAgv(cmdObj) {
    if (!agvSocket || agvSocket.readyState !== WebSocket.OPEN) {
        throw new Error('Chua ket noi duoc AGVServer.cpp');
    }
    agvSocket.send(JSON.stringify(cmdObj));
}

// Đồng bộ "logs" mà C++ Core gửi kèm mỗi lần broadcast vào SQLite (system_log), và phát
// hiện các sự kiện "sự cố xe" để đếm KPI. insertSystemLog() trả về false nếu mục này đã
// lưu trước đó (theo core_seq) -- tránh xử lý trùng do C++ Core gửi lại các mục cũ trong
// mỗi lần broadcast (~700ms/lần, "logs" luôn chứa ~20 mục gần nhất, không phải mục mới).
function persistSystemLogs(logs) {
    if (!Array.isArray(logs)) return;
    for (const entry of logs) {
        if (!entry || typeof entry.msg !== 'string') continue;
        const isNew = db.insertSystemLog(entry.seq, entry.time, entry.msg);
        if (!isNew) continue;
        const m = INCIDENT_RE.exec(entry.msg);
        if (m) db.insertAgvError(m[1], entry.msg);
    }
}

function onAgvState(state) {
    latestState = state;
    const agvs = (state.fleet && state.fleet.agvs) || [];

    for (const agv of agvs) {
        db.logAgvEventIfChanged(agv.id, agv.node, agv.battery, agv.state);

        const prevTarget = previousAgvTargets.get(agv.id) || '';
        const newTarget = agv.target || '';

        if (newTarget && newTarget !== prevTarget) {
            const pendingTask = db.findPendingTaskByTarget(newTarget);
            if (pendingTask) db.markTaskDispatched(pendingTask.id, agv.id);
        }

        if (prevTarget && !newTarget) {
            const dispatchedTask = db.findDispatchedTaskByTargetAgv(prevTarget, agv.id);
            if (dispatchedTask) db.markTaskCompleted(dispatchedTask.id);
        }
        previousAgvTargets.set(agv.id, newTarget);
    }

    persistSystemLogs(state.logs);
    broadcastToBrowsers({ ...state, agv_connected: true });
}

const wss = new WebSocket.Server({ server, path: '/ws' });
wss.on('connection', ws => {
    ws.send(JSON.stringify({ ...latestState, agv_connected: agvConnected }));
});

function broadcastToBrowsers(payload) {
    const text = JSON.stringify(payload);
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) client.send(text);
    });
}

function requireAgvOk(res) {
    if (!agvConnected) {
        res.status(503).json({ error: 'AGVServer.cpp chua ket noi duoc.' });
        return false;
    }
    return true;
}

app.get('/api/health', (req, res) => {
    res.json({ ok: true, backend: 'online', agv_connected: agvConnected, agv_ws_url: AGV_WS_URL, http_port: Number(PORT), time: new Date().toISOString() });
});

app.get('/api/state', (req, res) => res.json({ ...latestState, agv_connected: agvConnected }));
app.get('/api/map', (req, res) => res.json(latestState.map));
app.get('/api/mutex', (req, res) => res.json(latestState.mutex));
app.get('/api/agvs', (req, res) => res.json(latestState.fleet.agvs));

// ĐÃ BỎ: POST /api/agvs (thêm AGV thủ công từ dashboard). Từ giờ AGV chỉ được
// C++ Core tự động thêm khi ESP32 thật của xe gửi telemetry đầu tiên qua MQTT —
// xem AGVserver.cpp -> autoRegisterAgvIfNeeded(). Endpoint xoá xe (DELETE) vẫn
// giữ lại để người vận hành có thể gỡ 1 xe (VD đã tắt ESP32 hẳn) khỏi danh sách.
app.delete('/api/agvs/:id', (req, res) => {
    if (!requireAgvOk(res)) return;
    try {
        sendCommandToAgv({ cmd: 'remove_agv', id: req.params.id });
        res.json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.post('/api/agvs/:id/goto', (req, res) => {
    if (!requireAgvOk(res)) return;
    const agvId = req.params.id;
    const { target } = req.body;
    if (!target) return res.status(400).json({ error: 'Thieu target' });

    try {
        sendCommandToAgv({ cmd: 'direct_goto', agv_id: agvId, target });
        const taskId = db.createDispatchedTask(target, 0, agvId);
        previousAgvTargets.set(agvId, target);
        res.status(202).json({ ok: true, task_id: taskId });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

// MỚI: xoá trạng thái LỖI của 1 xe sau khi đã xử lý ngoài đời (nút "Xoá lỗi" trên dashboard).
// FleetManager KHÔNG tự xoá lỗi khi có telemetry mới -- phải xác nhận thủ công qua lệnh này.
app.post('/api/agvs/:id/clear-error', (req, res) => {
    if (!requireAgvOk(res)) return;
    try {
        sendCommandToAgv({ cmd: 'clear_error', id: req.params.id });
        res.status(202).json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.get('/api/agvs/:id/history', (req, res) => {
    const limit = Math.min(parseInt(req.query.limit) || 200, 1000);
    res.json(db.getAgvHistory(req.params.id, limit));
});

app.get('/api/tasks', (req, res) => res.json(latestState.fleet.pending_tasks));
app.get('/api/tasks/history', (req, res) => {
    const limit = Math.min(parseInt(req.query.limit) || 200, 1000);
    res.json(db.getTaskHistory(limit));
});

app.post('/api/tasks', (req, res) => {
    if (!requireAgvOk(res)) return;
    const { target, priority } = req.body;
    if (!target) return res.status(400).json({ error: 'Thieu target' });

    try {
        sendCommandToAgv({ cmd: 'add_task', target, priority: parseInt(priority) || 0 });
        db.createTask(target, parseInt(priority) || 0);
        res.status(202).json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

// MỚI: huỷ mọi task đang chờ/đang chạy hướng tới 1 node đích (nút "Huỷ" trên hàng đợi task).
// Cập nhật SQLite NGAY (đánh dấu 'cancelled') trước khi C++ Core kịp phản hồi qua WebSocket,
// để tránh bị logic "target rỗng -> completed" bên dưới (onAgvState) ghi đè nhầm thành hoàn
// thành bình thường -- xem chú thích tại db.cancelOpenTasksByTarget().
app.post('/api/tasks/cancel', (req, res) => {
    if (!requireAgvOk(res)) return;
    const { target } = req.body;
    if (!target) return res.status(400).json({ error: 'Thieu target' });

    try {
        sendCommandToAgv({ cmd: 'cancel_task', target });
        const changed = db.cancelOpenTasksByTarget(target);
        res.status(202).json({ ok: true, cancelled_in_db: changed });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

// MỚI: xem lại nhật ký hệ thống đã lưu (không chỉ ~20 mục gần nhất mà C++ Core phát realtime).
app.get('/api/logs', (req, res) => {
    const limit = Math.min(parseInt(req.query.limit) || 100, 1000);
    res.json(db.getRecentSystemLogs(limit));
});

app.post('/api/map/obstacle', (req, res) => {
    if (!requireAgvOk(res)) return;
    const { node, blocked } = req.body;
    if (!node) return res.status(400).json({ error: 'Thieu node' });

    try {
        const isBlocked = blocked !== false;
        sendCommandToAgv({ cmd: 'set_obstacle', node, blocked: isBlocked });
        db.logObstacle(node, isBlocked);

        if (!latestState.map.obstacles) latestState.map.obstacles = [];

        if (isBlocked) {
            if (!latestState.map.obstacles.includes(node)) latestState.map.obstacles.push(node);
        } else {
            latestState.map.obstacles = latestState.map.obstacles.filter(n => n !== node);
        }

        broadcastToBrowsers({ ...latestState, agv_connected: true });
        res.status(202).json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.get('/api/map/obstacles/history', (req, res) => {
    const limit = Math.min(parseInt(req.query.limit) || 200, 1000);
    res.json(db.getObstacleHistory(limit));
});

app.post('/api/map/node', (req, res) => {
    if (!requireAgvOk(res)) return;
    const { id, x, y, type } = req.body;
    if (!id) return res.status(400).json({ error: 'Thieu id node' });

    try {
        sendCommandToAgv({ cmd: 'add_node', id, x: x || 0, y: y || 0, type: type || 'waypoint' });
        res.status(202).json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.delete('/api/map/node/:id', (req, res) => {
    if (!requireAgvOk(res)) return;
    try {
        sendCommandToAgv({ cmd: 'remove_node', id: req.params.id });
        res.json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.post('/api/map/edge', (req, res) => {
    if (!requireAgvOk(res)) return;
    const { from, to, weight, type, path_class, two_way } = req.body;
    if (!from || !to) return res.status(400).json({ error: 'Thieu from/to' });

    try {
        sendCommandToAgv({ cmd: 'add_edge', from, to, weight: weight || 1, type: type || 'main', path_class: path_class || '', two_way: !!two_way });
        res.status(202).json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.delete('/api/map/edge', (req, res) => {
    if (!requireAgvOk(res)) return;
    const { from, to } = req.body;
    if (!from || !to) return res.status(400).json({ error: 'Thieu from/to' });

    try {
        sendCommandToAgv({ cmd: 'remove_edge', from, to });
        res.json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

// "Lưu nhanh": ghi đè file làm việc CỐ ĐỊNH của C++ Core (data/map_layout.json trên máy chạy
// AGVserver.cpp -- Core tự nạp lại đúng file này mỗi khi khởi động lại). Đồng thời cũng lưu 1
// bản snapshot có tên tự động vào SQLite để giữ lịch sử -- xem thêm ở "Lưu mới" (đặt tên) bên
// dưới, đó mới là nơi lưu trữ lâu dài/duyệt lại được nhiều phiên bản.
app.post('/api/map/save', (req, res) => {
    if (!requireAgvOk(res)) return;
    try {
        const savePath = req.body.path || CORE_MAP_SAVE_PATH;
        sendCommandToAgv({ cmd: 'save_map', path: savePath });
        const name = req.body.name || `Tu dong ${new Date().toLocaleString('vi-VN')}`;
        const id = db.saveMapSnapshot(latestState.map, name);
        res.json({ ok: true, id, name, path: savePath });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.get('/api/map/snapshots/latest', (req, res) => {
    const snap = db.getLatestMapSnapshot();
    if (!snap) return res.status(404).json({ error: 'Chua co snapshot nao duoc luu' });
    res.json(snap);
});

app.get('/api/map/snapshots', (req, res) => res.json(db.getAllMapSnapshots()));

// "Lưu mới": chỉ lưu 1 bản snapshot có tên vào SQLite (KHÔNG đụng tới file của C++ Core) --
// dùng để giữ nhiều phiên bản bản đồ (VD "Ca sáng", "Ca tối") và có thể nạp lại bất kỳ lúc nào
// qua "Nạp vào Server Đang Chạy" mà không sợ mất bản đang chạy.
app.post('/api/map/snapshots', (req, res) => {
    try {
        const name = (req.body && req.body.name && req.body.name.trim())
            || `Tu dong ${new Date().toLocaleString('vi-VN')}`;
        const id = db.saveMapSnapshot(latestState.map, name);
        res.status(201).json({ ok: true, id, name });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.get('/api/map/snapshots/:id', (req, res) => {
    const snap = db.getMapSnapshotById(req.params.id);
    if (!snap) return res.status(404).json({ error: 'Khong tim thay ban do' });
    res.json(snap);
});

app.post('/api/map/snapshots/:id/apply', (req, res) => {
    if (!requireAgvOk(res)) return;
    const snap = db.getMapSnapshotById(req.params.id);
    if (!snap) return res.status(404).json({ error: 'Khong tim thay ban do' });

    try {
        sendCommandToAgv({ cmd: 'load_map_data', map: snap.map });
        res.json({ ok: true });
    } catch (error) {
        res.status(503).json({ error: error.message });
    }
});

app.delete('/api/map/snapshots/:id', (req, res) => {
    try {
        db.deleteMapSnapshot(req.params.id);
        res.json({ ok: true });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

app.get('/api/kpi/latest', (req, res) => {
    const report = db.getLatestKpiReport();
    if (!report) return res.status(404).json({ error: 'Chua co bao cao KPI nao' });
    res.json(report);
});

app.get('/api/kpi/history', (req, res) => {
    const limit = Math.min(parseInt(req.query.limit) || 50, 200);
    res.json(db.getKpiHistory(limit));
});

app.post('/api/kpi/generate', (req, res) => {
    try {
        const report = db.computeKpiReport('manual');
        res.json({ ok: true, report });
    } catch (error) {
        res.status(500).json({ error: error.message });
    }
});

connectToAgvServer();

server.on('error', error => console.error('[HTTP] Khong the khoi dong server:', error.message));
server.listen(PORT, () => {
    console.log(`[HTTP] Backend API + Dashboard: http://localhost:${PORT}`);
    console.log(`[WS] Realtime Browser: ws://localhost:${PORT}/ws`);
    console.log(`[LINK] AGVServer.cpp: ${AGV_WS_URL}`);
    console.log(`[MAP] File ban do cua C++ Core: ${CORE_MAP_SAVE_PATH} (tren may chay AGVserver.cpp)`);
});