// ============================================================================
// db.js — Database SQLite cho hệ thống AGV
// ============================================================================
const Database = require('better-sqlite3');
const path = require('path');

const db = new Database(path.join(__dirname, 'agv.db'));
db.pragma('journal_mode = WAL');

db.exec(`
CREATE TABLE IF NOT EXISTS agv_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    agv_id TEXT NOT NULL,
    node TEXT,
    battery INTEGER,
    state TEXT,
    created_at TEXT DEFAULT (datetime('now'))
);
CREATE INDEX IF NOT EXISTS idx_agv_events_agv ON agv_events(agv_id, created_at);

CREATE TABLE IF NOT EXISTS task_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    target_node TEXT NOT NULL,
    priority INTEGER DEFAULT 0,
    status TEXT DEFAULT 'pending',
    assigned_agv TEXT,
    created_at TEXT DEFAULT (datetime('now')),
    completed_at TEXT
);
CREATE INDEX IF NOT EXISTS idx_task_log_status ON task_log(status);

CREATE TABLE IF NOT EXISTS obstacle_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    node_id TEXT NOT NULL,
    blocked INTEGER NOT NULL,
    created_at TEXT DEFAULT (datetime('now'))
);

CREATE TABLE IF NOT EXISTS map_snapshots (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT DEFAULT 'Khong ten',
    map_json TEXT NOT NULL,
    saved_at TEXT DEFAULT (datetime('now'))
);

CREATE TABLE IF NOT EXISTS kpi_reports (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    reason TEXT NOT NULL,
    report_json TEXT NOT NULL,
    created_at TEXT DEFAULT (datetime('now'))
);

-- MỚI: nhật ký sự kiện toàn hệ thống, đồng bộ từ "logs" mà C++ Core phát kèm
-- mỗi lần broadcast trạng thái (xem AGVserver.cpp / pushLog). core_seq là số
-- thứ tự C++ Core gán cho từng sự kiện -> dùng UNIQUE index để INSERT OR IGNORE,
-- tránh ghi trùng khi C++ Core gửi lại cùng 1 mục nhiều lần trong các lần
-- broadcast liên tiếp (mỗi ~700ms, "logs" luôn chứa ~20 mục gần nhất).
CREATE TABLE IF NOT EXISTS system_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    core_seq INTEGER,
    time_label TEXT,
    message TEXT NOT NULL,
    created_at TEXT DEFAULT (datetime('now'))
);
CREATE UNIQUE INDEX IF NOT EXISTS idx_system_log_core_seq ON system_log(core_seq);

-- MỚI: chỉ riêng các sự kiện "sự cố xe" (lỗi / mất ACK lệnh / mất kết nối) được
-- trích ra từ system_log để tính KPI "Sự cố xe" trên dashboard.
CREATE TABLE IF NOT EXISTS agv_error_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    agv_id TEXT,
    message TEXT,
    created_at TEXT DEFAULT (datetime('now'))
);
`);

try {
  db.prepare(`SELECT name FROM map_snapshots LIMIT 1`).get();
} catch (error) {
  db.exec(`ALTER TABLE map_snapshots ADD COLUMN name TEXT DEFAULT 'Khong ten'`);
}

const stmts = {
  insertAgvEvent: db.prepare(`INSERT INTO agv_events (agv_id, node, battery, state) VALUES (?, ?, ?, ?)`),
  lastAgvEvent: db.prepare(`SELECT * FROM agv_events WHERE agv_id = ? ORDER BY id DESC LIMIT 1`),
  agvHistory: db.prepare(`SELECT * FROM agv_events WHERE agv_id = ? ORDER BY id DESC LIMIT ?`),

  insertTask: db.prepare(`INSERT INTO task_log (target_node, priority, status) VALUES (?, ?, 'pending')`),
  insertDispatchedTask: db.prepare(`INSERT INTO task_log (target_node, priority, status, assigned_agv) VALUES (?, ?, 'dispatched', ?)`),
  markTaskDispatched: db.prepare(`UPDATE task_log SET status = 'dispatched', assigned_agv = ? WHERE id = ?`),
  markTaskCompleted: db.prepare(`UPDATE task_log SET status = 'completed', completed_at = datetime('now') WHERE id = ?`),
  cancelOpenTasksByTarget: db.prepare(`UPDATE task_log SET status = 'cancelled', completed_at = datetime('now') WHERE target_node = ? AND status IN ('pending', 'dispatched')`),
  pendingOrDispatchedTasks: db.prepare(`SELECT * FROM task_log WHERE status IN ('pending', 'dispatched') ORDER BY id ASC`),
  taskHistory: db.prepare(`SELECT * FROM task_log ORDER BY id DESC LIMIT ?`),
  findPendingTaskByTarget: db.prepare(`SELECT * FROM task_log WHERE target_node = ? AND status = 'pending' ORDER BY id ASC LIMIT 1`),
  findDispatchedTaskByTargetAgv: db.prepare(`SELECT * FROM task_log WHERE target_node = ? AND status = 'dispatched' AND assigned_agv = ? ORDER BY id ASC LIMIT 1`),

  insertObstacle: db.prepare(`INSERT INTO obstacle_log (node_id, blocked) VALUES (?, ?)`),
  obstacleHistory: db.prepare(`SELECT * FROM obstacle_log ORDER BY id DESC LIMIT ?`),

  insertMapSnapshot: db.prepare(`INSERT INTO map_snapshots (name, map_json) VALUES (?, ?)`),
  latestMapSnapshot: db.prepare(`SELECT * FROM map_snapshots ORDER BY id DESC LIMIT 1`),
  allMapSnapshots: db.prepare(`SELECT id, name, saved_at FROM map_snapshots ORDER BY id DESC`),
  mapSnapshotById: db.prepare(`SELECT * FROM map_snapshots WHERE id = ?`),
  deleteMapSnapshot: db.prepare(`DELETE FROM map_snapshots WHERE id = ?`),

  insertKpi: db.prepare(`INSERT INTO kpi_reports (reason, report_json) VALUES (?, ?)`),
  kpiCreatedAt: db.prepare(`SELECT created_at FROM kpi_reports WHERE id = ?`),
  latestKpi: db.prepare(`SELECT * FROM kpi_reports ORDER BY id DESC LIMIT 1`),
  kpiHistory: db.prepare(`SELECT * FROM kpi_reports ORDER BY id DESC LIMIT ?`),

  insertSystemLog: db.prepare(`INSERT OR IGNORE INTO system_log (core_seq, time_label, message) VALUES (?, ?, ?)`),
  recentSystemLogs: db.prepare(`SELECT * FROM system_log ORDER BY id DESC LIMIT ?`),
  insertAgvError: db.prepare(`INSERT INTO agv_error_log (agv_id, message) VALUES (?, ?)`),

  now: db.prepare(`SELECT datetime('now') AS t`),
  minAgvEventAt: db.prepare(`SELECT MIN(created_at) AS t FROM agv_events`),
  minTaskAt: db.prepare(`SELECT MIN(created_at) AS t FROM task_log`),
  countTasksInWindow: db.prepare(`SELECT COUNT(*) AS n FROM task_log WHERE created_at >= ? AND created_at <= ?`),
  countTasksByStatusInWindow: db.prepare(`SELECT COUNT(*) AS n FROM task_log WHERE status = ? AND created_at >= ? AND created_at <= ?`),
  avgCompletionSecondsInWindow: db.prepare(`
    SELECT AVG((julianday(completed_at) - julianday(created_at)) * 86400.0) AS sec
    FROM task_log WHERE status = 'completed' AND completed_at IS NOT NULL AND created_at >= ? AND created_at <= ?
  `),
  countObstacleEventsInWindow: db.prepare(`SELECT COUNT(*) AS n FROM obstacle_log WHERE blocked = 1 AND created_at >= ? AND created_at <= ?`),
  countTelemetryEventsInWindow: db.prepare(`SELECT COUNT(*) AS n FROM agv_events WHERE created_at >= ? AND created_at <= ?`),
  countAgvsInWindow: db.prepare(`SELECT COUNT(DISTINCT agv_id) AS n FROM agv_events WHERE created_at >= ? AND created_at <= ?`),
  countAgvErrorsInWindow: db.prepare(`SELECT COUNT(*) AS n FROM agv_error_log WHERE created_at >= ? AND created_at <= ?`),
};

module.exports = {
  db,

  logAgvEventIfChanged(agvId, node, battery, state) {
    const last = stmts.lastAgvEvent.get(agvId);
    if (last && last.node === node && last.battery === battery && last.state === state) return;
    stmts.insertAgvEvent.run(agvId, node, battery, state);
  },

  getAgvHistory(agvId, limit = 200) {
    return stmts.agvHistory.all(agvId, limit);
  },

  createTask(targetNode, priority) {
    return stmts.insertTask.run(targetNode, priority).lastInsertRowid;
  },

  createDispatchedTask(targetNode, priority, agvId) {
    return stmts.insertDispatchedTask.run(targetNode, priority, agvId).lastInsertRowid;
  },

  getOpenTasks() {
    return stmts.pendingOrDispatchedTasks.all();
  },

  markTaskDispatched(id, agvId) {
    stmts.markTaskDispatched.run(agvId, id);
  },

  markTaskCompleted(id) {
    stmts.markTaskCompleted.run(id);
  },

  // Đánh dấu 'cancelled' mọi task (pending/dispatched) đang nhắm tới 1 target -- gọi ngay khi
  // nhận yêu cầu huỷ từ dashboard, TRƯỚC khi C++ Core kịp phản hồi qua WebSocket, để tránh bị
  // logic "target rỗng -> completed" trong onAgvState() ghi đè nhầm thành 'completed'.
  cancelOpenTasksByTarget(target) {
    return stmts.cancelOpenTasksByTarget.run(target).changes;
  },

  getTaskHistory(limit = 200) {
    return stmts.taskHistory.all(limit);
  },

  findPendingTaskByTarget(target) {
    return stmts.findPendingTaskByTarget.get(target);
  },

  findDispatchedTaskByTargetAgv(target, agvId) {
    return stmts.findDispatchedTaskByTargetAgv.get(target, agvId);
  },

  logObstacle(nodeId, blocked) {
    stmts.insertObstacle.run(nodeId, blocked ? 1 : 0);
  },

  getObstacleHistory(limit = 200) {
    return stmts.obstacleHistory.all(limit);
  },

  saveMapSnapshot(mapJson, name = 'Khong ten') {
    return stmts.insertMapSnapshot.run(name || 'Khong ten', JSON.stringify(mapJson)).lastInsertRowid;
  },

  getLatestMapSnapshot() {
    const row = stmts.latestMapSnapshot.get();
    return row ? JSON.parse(row.map_json) : null;
  },

  getAllMapSnapshots() {
    return stmts.allMapSnapshots.all();
  },

  getMapSnapshotById(id) {
    const row = stmts.mapSnapshotById.get(Number(id));
    if (!row) return null;
    return { id: row.id, name: row.name, saved_at: row.saved_at, map: JSON.parse(row.map_json) };
  },

  deleteMapSnapshot(id) {
    stmts.deleteMapSnapshot.run(Number(id));
  },

  // MỚI: ghi 1 mục nhật ký hệ thống (đồng bộ từ C++ Core). Trả về true nếu đây là mục MỚI
  // (chưa từng lưu, phân biệt theo core_seq) -- dùng để tránh xử lý trùng (VD đếm sự cố xe 2 lần).
  insertSystemLog(coreSeq, timeLabel, message) {
    const info = stmts.insertSystemLog.run(coreSeq, timeLabel, message);
    return info.changes > 0;
  },

  getRecentSystemLogs(limit = 100) {
    return stmts.recentSystemLogs.all(limit);
  },

  insertAgvError(agvId, message) {
    stmts.insertAgvError.run(agvId, message);
  },

  // Báo cáo KPI theo CỬA SỔ THỜI GIAN: từ lúc kết thúc báo cáo TRƯỚC ĐÓ (period_end) tới hiện
  // tại -- giống 1 "báo cáo ca làm việc" -- thay vì luỹ kế từ đầu mãi mãi. Nếu chưa từng có báo
  // cáo nào, cửa sổ bắt đầu từ sự kiện AGV/task sớm nhất từng ghi nhận.
  // Định dạng thời gian: LUÔN dùng SQLite datetime('now') ("YYYY-MM-DD HH:MM:SS", giờ UTC) cho
  // period_start/period_end, để so sánh chuỗi ">=""<=" với cột created_at (cùng định dạng) luôn
  // đúng theo thời gian thực -- KHÔNG dùng JS toISOString() (khác định dạng, so sánh chuỗi sẽ sai).
  computeKpiReport(reason = 'manual') {
    const periodEnd = stmts.now.get().t;
    const prevRow = stmts.latestKpi.get();
    const prevReport = prevRow ? JSON.parse(prevRow.report_json) : null;
    const periodStart = (prevReport && prevReport.period_end)
      ? prevReport.period_end
      : (stmts.minAgvEventAt.get().t || stmts.minTaskAt.get().t || periodEnd);

    const totalTasks = stmts.countTasksInWindow.get(periodStart, periodEnd).n;
    const completedTasks = stmts.countTasksByStatusInWindow.get('completed', periodStart, periodEnd).n;
    const dispatchedTasks = stmts.countTasksByStatusInWindow.get('dispatched', periodStart, periodEnd).n;
    const pendingTasks = stmts.countTasksByStatusInWindow.get('pending', periodStart, periodEnd).n;
    const cancelledTasks = stmts.countTasksByStatusInWindow.get('cancelled', periodStart, periodEnd).n;

    const avgSec = stmts.avgCompletionSecondsInWindow.get(periodStart, periodEnd).sec;
    const obstacleEvents = stmts.countObstacleEventsInWindow.get(periodStart, periodEnd).n;
    const telemetryEvents = stmts.countTelemetryEventsInWindow.get(periodStart, periodEnd).n;
    const agvCount = stmts.countAgvsInWindow.get(periodStart, periodEnd).n;
    const agvErrorEvents = stmts.countAgvErrorsInWindow.get(periodStart, periodEnd).n;

    const report = {
      reason,
      period_start: periodStart,
      period_end: periodEnd,
      total_tasks: totalTasks,
      completed_tasks: completedTasks,
      dispatched_tasks: dispatchedTasks,
      pending_tasks: pendingTasks,
      cancelled_tasks: cancelledTasks,
      avg_completion_seconds: avgSec == null ? null : Number(avgSec.toFixed(1)),
      agv_error_events: agvErrorEvents,
      obstacle_events: obstacleEvents,
      telemetry_events: telemetryEvents,
      agv_count: agvCount,
    };

    const info = stmts.insertKpi.run(reason, JSON.stringify(report));
    const createdAt = stmts.kpiCreatedAt.get(info.lastInsertRowid).created_at;
    return { id: info.lastInsertRowid, created_at: createdAt, ...report };
  },

  getLatestKpiReport() {
    const row = stmts.latestKpi.get();
    if (!row) return null;
    return { id: row.id, created_at: row.created_at, ...JSON.parse(row.report_json) };
  },

  getKpiHistory(limit = 50) {
    return stmts.kpiHistory.all(limit).map(row => ({
      id: row.id,
      created_at: row.created_at,
      ...JSON.parse(row.report_json),
    }));
  }
};