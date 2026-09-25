#pragma once
// ============================================================================
// FleetManager.hpp — Quản lý đội xe & Bộ điều phối nhiệm vụ (Dispatcher)
//
// PHIÊN BẢN NÂNG CẤP "BƯỚC 1 — ĐỘ TIN CẬY" so với bản gốc:
//
//  [1] HEARTBEAT / OFFLINE
//      - tick() quét mọi xe: im lặng quá offlineTimeout_ (mặc định 10s) ->
//        state = OFFLINE, task đang làm được TRẢ LẠI hàng đợi (xe khác nhận),
//        mutex zone xe đó đang giữ được nhả (có thể tắt bằng
//        setReleaseZonesOnOffline(false)) VÀ quyền zone được trao thẳng cho xe
//        đang xếp hàng kế tiếp (publish mutex_grant) — nếu không xe chờ sẽ đứng
//        mãi. Các xe khác đang có đường đi qua node của xe chết được replan.
//      - Xe OFFLINE gửi telemetry lại -> về IDLE, server gửi "WAIT" để xe dừng
//        chờ lệnh mới (vì kế hoạch cũ của nó đã bị huỷ).
//
//  [2] LỖI & KHÔNG LÀM MẤT TASK
//      - reportError(): trả task về hàng đợi, xoá đường đi, gửi "STOP".
//      - Telemetry KHÔNG còn tự xoá trạng thái ERROR. Phải gọi clearError(id)
//        (lệnh web "clear_error") sau khi người vận hành xử lý xong.
//      - directGoto() / removeAGV() không còn làm mất task đang chạy: task cũ
//        được trả về hàng đợi.
//      - Xe BLOCKED giờ được tick() thử lại mỗi vòng (resumeBlocked()) và
//        tự chạy tiếp khi có đường (trước đây kẹt vĩnh viễn).
//
//  [3] THEO DÕI TIẾN ĐỘ CHỊU LỖI RFID
//      - Nhận telemetry tại node nằm SÂU hơn trong activePath (xe bỏ lỡ 1 thẻ)
//        -> cắt bỏ mọi node đã qua, task vẫn hoàn thành.
//      - Nhận node không thuộc đường đi và xe đã di chuyển (lệch đường)
//        -> tự replan từ vị trí thật.
//      - Node RFID không tồn tại trên map -> bỏ qua vị trí, vẫn cập nhật pin/lastSeen.
//
//  [4] XE ĐỨNG YÊN CŨNG LÀ VẬT CẢN
//      - Khi tìm đường, tránh cả currentNode của mọi xe khác (IDLE, đang sạc,
//        lỗi, offline...) chứ không chỉ activePath. Nếu không còn đường nào
//        khác thì vẫn fallback đi xuyên (chấp nhận xếp hàng qua mutex zone).
//
//  Khác: checkBatteryOne bỏ qua xe OFFLINE/ERROR; xe đã đứng sẵn tại trạm sạc
//  chuyển thẳng sang CHARGING; chỉ notify()/dispatch khi thực sự có thay đổi
//  (tránh broadcast dồn dập mỗi tick); thêm callback sự kiện (setEventCallback).
//
// PHIÊN BẢN NÂNG CẤP "BƯỚC 2 — LỆNH TIN CẬY & QUẢN LÝ TRẠM SẠC":
//
//  [5] LỆNH TIN CẬY (MQTT ACK + GỬI LẠI)
//      *** THAY ĐỔI GIAO THỨC (breaking change, cần cập nhật firmware ESP32) ***
//      Mọi lệnh xuống xe (GOTO/STOP/WAIT/CHARGE_START) giờ LUÔN là JSON, có
//      thêm "cmd_id" (số tăng dần, duy nhất toàn hệ thống):
//        {"cmd":"GOTO","cmd_id":42,"path":[...],"target":"..."}
//        {"cmd":"STOP","cmd_id":43}  {"cmd":"WAIT","cmd_id":44}
//        {"cmd":"CHARGE_START","cmd_id":45}
//      ESP32 PHẢI publish xác nhận tại "agv/{id}/ack" với payload = cmd_id
//      (dạng số, VD "42") ngay khi nhận được lệnh (không cần đợi thực hiện
//      xong). Nếu ESP32 nhận lại cùng 1 cmd_id đã xử lý (do server gửi lại),
//      nên bỏ qua để tránh lặp hành động, nhưng vẫn phải ack lại.
//      Server (setAckTimeout, mặc định 3s / setMaxRetries, mặc định 3 lần):
//        không thấy ack trong ackTimeout_ -> gửi lại nguyên văn cùng cmd_id;
//        hết maxRetries_ mà vẫn im lặng -> coi là mất liên lạc lệnh, TRẢ task
//        về hàng đợi và chuyển xe sang AGV_ERROR (cần clearError() sau khi
//        khắc phục mạng/firmware). Lệnh mới gửi cho 1 xe luôn thay thế lệnh
//        cũ đang chờ ack của chính xe đó (không cộng dồn).
//
//  [6] QUẢN LÝ TRẠM SẠC
//      - Đặt chỗ (reservation): khi cử xe đi sạc, trạm được "giữ chỗ" ngay
//        (chargeReservation_) để xe pin thấp khác không cùng nhắm tới trạm
//        đó trong lúc đang trên đường tới (trước khi vật lý chiếm node).
//        Hết sạc / hủy kế hoạch / mất kết nối / bị xoá -> nhả chỗ ngay.
//      - Hysteresis pin (chống dao động nhận/hủy task liên tục quanh 1
//        ngưỡng): setChargeThresholds(low, resume, targetPercent).
//        pin <= low -> về sạc; CHỈ nhận task mới khi pin >= resume (resume
//        phải > low, mặc định low=20, resume=40).
//      - Pin sau khi sạc dùng giá trị THẬT nếu ESP32 báo kèm theo
//        "charging_done" (payload = % pin cuối, có thể rỗng), nếu không có
//        thì dùng targetPercent (mặc định 90) thay vì luôn ép cứng 100.
//
// Thiết kế giữ nguyên: KHÔNG phụ thuộc MQTT/WebSocket; gửi lệnh qua
// std::function do Server tiêm vào. Mọi thao tác đổi state giữ mtx_ trong
// 1 block riêng, rồi MỚI gọi notify()/dispatch/callback bên ngoài block.
// (Ngoại lệ có chủ đích như bản gốc: publish_ được gọi trong assignPath khi
//  đang giữ mtx_ — callback publish của Server KHÔNG được gọi ngược vào
//  FleetManager.)
// ============================================================================
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <mutex>
#include <functional>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include "json.hpp"
#include "MapGraph.hpp"
#include "MutexArbiter.hpp"

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

enum class AGVState { IDLE, WORKING, CHARGING_EN_ROUTE, CHARGING, BLOCKED, AGV_ERROR, OFFLINE };

inline std::string toString(AGVState s) {
    switch (s) {
        case AGVState::IDLE: return "IDLE";
        case AGVState::WORKING: return "WORKING";
        case AGVState::CHARGING_EN_ROUTE: return "CHARGING_EN_ROUTE";
        case AGVState::CHARGING: return "CHARGING";
        case AGVState::BLOCKED: return "BLOCKED";
        case AGVState::AGV_ERROR: return "ERROR";
        default: return "OFFLINE";
    }
}

struct AGV {
    std::string id;
    std::string currentNode;
    int battery = 100;
    AGVState state = AGVState::OFFLINE;
    std::vector<std::string> activePath; // các node còn lại phía trước
    std::string finalTarget;             // đích cuối cùng của task hiện tại
    bool isPriorityVehicle = false;      // xe ưu tiên (VD: chở hàng gấp)
    AGVState blockedResume = AGVState::IDLE; // trạng thái sẽ quay lại khi hết BLOCKED
    Clock::time_point lastSeen = Clock::now();
};

struct Task {
    std::string targetNode;
    int priority = 0; // cao hơn = ưu tiên hơn
    long seq = 0;      // thứ tự tạo, dùng để FIFO khi bằng priority
};

class FleetManager {
public:
    FleetManager(MapGraph& map, MutexArbiter& arbiter) : map_(map), arbiter_(arbiter) {}

    // Gán bởi Server: gửi lệnh MQTT xuống AGV, topic dạng "agv/{id}/command"
    void setPublishCallback(std::function<void(const std::string&, const std::string&)> cb) { publish_ = std::move(cb); }
    // Gán bởi Server: được gọi mỗi khi state đổi -> Server đẩy lên WebSocket
    void setStateChangedCallback(std::function<void()> cb) { onChanged_ = std::move(cb); }
    // Gán bởi Server: nhận thông báo sự kiện dạng text (offline, lỗi, trả task...) để log/cảnh báo
    void setEventCallback(std::function<void(const std::string&)> cb) { onEvent_ = std::move(cb); }

    // Tương thích ngược: chỉ đặt ngưỡng "pin thấp -> đi sạc". Nên dùng setChargeThresholds() thay thế.
    void setBatteryThresholds(int lowPercent) {
        std::lock_guard<std::mutex> lk(mtx_);
        lowBattery_ = lowPercent;
        if (resumeBattery_ <= lowBattery_) resumeBattery_ = lowBattery_ + 1;
    }
    // lowPercent: pin <= mức này -> tự động về sạc.
    // resumePercent: CHỈ nhận task mới khi pin >= mức này (phải > lowPercent, tránh nhận-rồi-lại-huỷ
    //   liên tục quanh 1 ngưỡng, VD pin 21% nhận task rồi vài phút sau lại bị huỷ để đi sạc).
    // targetPercent: pin coi là "sạc xong" nếu ESP32 không báo kèm % pin thật lúc charging_done.
    void setChargeThresholds(int lowPercent, int resumePercent, int targetPercent = 90) {
        std::lock_guard<std::mutex> lk(mtx_);
        lowBattery_ = lowPercent;
        resumeBattery_ = (resumePercent > lowPercent) ? resumePercent : lowPercent + 1;
        chargeTargetPercent_ = targetPercent;
    }
    // Bao lâu chờ ACK trước khi gửi lại cùng 1 lệnh (mặc định 3000ms)
    void setAckTimeout(int milliseconds) {
        std::lock_guard<std::mutex> lk(mtx_);
        ackTimeout_ = std::chrono::milliseconds(milliseconds > 0 ? milliseconds : 100);
    }
    // Số lần gửi lại tối đa trước khi coi là mất liên lạc lệnh (mặc định 3)
    void setMaxRetries(int n) {
        std::lock_guard<std::mutex> lk(mtx_);
        maxRetries_ = n >= 0 ? n : 0;
    }
    // Sau bao nhiêu giây không nhận telemetry thì coi xe là OFFLINE (mặc định 10s)
    void setOfflineTimeout(int seconds) {
        std::lock_guard<std::mutex> lk(mtx_);
        offlineTimeout_ = std::chrono::seconds(seconds > 0 ? seconds : 1);
    }
    // true (mặc định): telemetry từ xe chưa đăng ký sẽ tự tạo xe. false: bỏ qua xe lạ.
    void setAutoRegister(bool on) {
        std::lock_guard<std::mutex> lk(mtx_);
        autoRegister_ = on;
    }
    // true (mặc định): xe mất kết nối sẽ bị nhả mutex zone để không treo cả xưởng.
    // Lưu ý: xe offline có thể vẫn đứng trong zone -> đặt false nếu ưu tiên an toàn hơn thông suốt.
    void setReleaseZonesOnOffline(bool on) {
        std::lock_guard<std::mutex> lk(mtx_);
        releaseZonesOnOffline_ = on;
    }

    // ---------------- Quản lý xe ----------------
    void addAGV(const std::string& id, const std::string& startNode, bool isPriority = false) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            AGV a; a.id = id; a.currentNode = startNode; a.state = AGVState::IDLE;
            a.isPriorityVehicle = isPriority; a.lastSeen = Clock::now();
            fleet_[id] = a;
        }
        notify();
        dispatchPending();
    }

    void removeAGV(const std::string& id) {
        bool requeued = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(id);
            if (it != fleet_.end()) {
                requeued = requeueWorkNoLock(it->second);
                fleet_.erase(it);
            }
        }
        releaseAgvZones(id);
        if (requeued) emit("AGV_" + id + " bi xoa khi dang co task -> tra task ve hang doi");
        notify();
        dispatchPending();
    }

    // ---------------- Nhận dữ liệu từ ESP32 (telemetry) ----------------
    // obstacleAhead = true nếu cảm biến AGV phát hiện vật cản ngay phía trước
    void updateTelemetry(const std::string& id, const std::string& rfid, int battery, bool obstacleAhead) {
        const bool nodeValid = map_.nodeExists(rfid);
        std::vector<std::string> toReplan;
        bool justFinishedCharging = false;
        bool recovered = false;
        bool unknownNode = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(id);
            if (it == fleet_.end()) {
                if (!autoRegister_) return;
                AGV a; a.id = id;               // state = OFFLINE -> nhánh recovered bên dưới đưa về IDLE
                it = fleet_.emplace(id, a).first;
            }
            AGV& agv = it->second;
            const std::string prevNode = agv.currentNode;

            agv.battery = battery;
            agv.lastSeen = Clock::now();
            if (nodeValid) agv.currentNode = rfid; else unknownNode = true;

            // Xe mất kết nối nay đã quay lại. (ERROR KHÔNG tự xoá — dùng clearError.)
            if (agv.state == AGVState::OFFLINE) {
                agv.state = AGVState::IDLE;
                agv.blockedResume = AGVState::IDLE;
                recovered = true;
                // Kế hoạch cũ đã bị huỷ khi xe offline -> bảo xe dừng chờ lệnh mới.
                // Gửi trong lúc đang giữ mtx_ (như assignPath) để pendingCmd_ nhất quán.
                sendCommand(agv, "WAIT");
            }

            // Cập nhật tiến độ đường đi (chịu được việc bỏ lỡ thẻ RFID)
            if (nodeValid && !agv.activePath.empty()) {
                auto pit = std::find(agv.activePath.begin(), agv.activePath.end(), rfid);
                if (pit != agv.activePath.end()) {
                    agv.activePath.erase(agv.activePath.begin(), pit + 1); // cắt tới node vừa qua
                } else if (rfid != prevNode) {
                    toReplan.push_back(id); // xe đã dịch chuyển nhưng không nằm trên đường -> lệch đường
                }
            }

            if (obstacleAhead && !agv.activePath.empty()) {
                map_.setObstacle(agv.activePath.front(), true);
                if (std::find(toReplan.begin(), toReplan.end(), id) == toReplan.end())
                    toReplan.push_back(id);
            }

            // Đã tới đích cuối cùng?
            if (nodeValid && agv.activePath.empty() && !agv.finalTarget.empty() && agv.currentNode == agv.finalTarget
                && (agv.state == AGVState::WORKING || agv.state == AGVState::CHARGING_EN_ROUTE)) {
                bool wasCharging = (agv.state == AGVState::CHARGING_EN_ROUTE);
                agv.finalTarget.clear();
                agv.state = wasCharging ? AGVState::CHARGING : AGVState::IDLE;
                justFinishedCharging = wasCharging;
                if (wasCharging) sendCommand(agv, "CHARGE_START"); // đã tới trạm -> báo xe bắt đầu sạc
            }
        }
        if (unknownNode) emit("AGV_" + id + " gui RFID khong ton tai tren map: " + rfid);
        if (recovered) emit("AGV_" + id + " ket noi lai (ONLINE)");
        (void)justFinishedCharging; // đã gửi lệnh ở trên trong lúc còn giữ mtx_
        for (auto& id2 : toReplan) replanAGV(id2); // né vật cản / lệch đường

        checkBatteryOne(id);
        dispatchPending();
        notify();
    }

    // Gọi khi AGV báo đã sạc xong (topic agv/{id}/charging_done).
    // finalBatteryPercent: % pin thật lúc ESP32 báo xong, nếu ESP32 không gửi kèm (payload rỗng)
    // thì truyền -1 để dùng chargeTargetPercent_ (mặc định 90) thay vì ép cứng 100 như bản cũ.
    void reportCharged(const std::string& id, int finalBatteryPercent = -1) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(id);
            if (it == fleet_.end()) return;
            AGV& agv = it->second;
            agv.battery = std::min(100, finalBatteryPercent >= 0 ? finalBatteryPercent : chargeTargetPercent_);
            agv.state = AGVState::IDLE;
            releaseStationReservationNoLock(id); // trả chỗ trạm sạc cho xe khác
        }
        notify();
        dispatchPending();
    }

    // AGV báo lỗi: huỷ kế hoạch, TRẢ task về hàng đợi cho xe khác, giữ ERROR đến khi clearError().
    void reportError(const std::string& id, const std::string& msg) {
        std::vector<std::string> toReplan;
        bool requeued = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(id);
            if (it == fleet_.end()) return;
            AGV& agv = it->second;
            requeued = requeueWorkNoLock(agv);
            agv.state = AGVState::AGV_ERROR;
            toReplan = agvsRoutedThroughNoLock(agv.currentNode, id); // xe lỗi đang chắn node này
            sendCommand(agv, "STOP");
        }
        emit("AGV_" + id + " BAO LOI: " + msg + (requeued ? " -> task duoc tra ve hang doi" : ""));
        for (auto& o : toReplan) replanAGV(o);
        notify();
        dispatchPending();
    }

    // Người vận hành xác nhận đã xử lý lỗi -> xe trở lại IDLE. Trả về true nếu có xe đang ERROR.
    bool clearError(const std::string& id) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(id);
            if (it == fleet_.end() || it->second.state != AGVState::AGV_ERROR) return false;
            it->second.state = AGVState::IDLE;
        }
        emit("AGV_" + id + " da duoc xoa loi (IDLE)");
        notify();
        dispatchPending();
        return true;
    }

    // ESP32 xác nhận đã nhận lệnh (topic agv/{id}/ack, payload = cmd_id). Ngăn resend không cần thiết.
    // Ack trễ (đến sau khi đã gửi lại hoặc đã hết hạn/đã báo lỗi) bị bỏ qua vì cmd_id không còn khớp.
    void reportAck(const std::string& id, long cmdId) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = pendingCmd_.find(id);
        if (it != pendingCmd_.end() && it->second.cmdId == cmdId) pendingCmd_.erase(it);
    }

    // Vật cản báo qua topic riêng (agv/{id}/obstacle), payload = nodeId phát hiện vật cản
    void reportObstacle(const std::string& agvId, const std::string& nodeId) {
        map_.setObstacle(nodeId, true);
        replanAGV(agvId);
        // Các xe khác đang có node này trên đường đi cũng phải replanning
        std::vector<std::string> others;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            others = agvsRoutedThroughNoLock(nodeId, agvId);
        }
        for (auto& id : others) replanAGV(id);
        notify();
    }
    void clearObstacle(const std::string& nodeId) {
        map_.setObstacle(nodeId, false);
        notify();
    }

    // ---------------- Giao task ----------------
    void addTask(const std::string& targetNode, int priority = 0) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            addTaskInternal(targetNode, priority);
        }
        dispatchPending();
    }

    // ---------------- Ra lệnh trực tiếp 1 xe cụ thể (bấm chọn trên map) ----------------
    // Khác addTask(): KHÔNG tự tìm "xe gần nhất", mà điều khiển ĐÚNG agvId do
    // người dùng chỉ định. Nếu xe đang làm task khác, task cũ được TRẢ VỀ hàng đợi.
    // Trả về "" nếu thành công, hoặc chuỗi mô tả lỗi nếu thất bại.
    std::string directGoto(const std::string& agvId, const std::string& targetNode) {
        if (!map_.nodeExists(targetNode)) return "Node dich khong ton tai: " + targetNode;
        bool requeued = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(agvId);
            if (it == fleet_.end())
                return "Khong tim thay AGV: " + agvId;
            AGV& agv = it->second;
            if (agv.state == AGVState::CHARGING || agv.state == AGVState::CHARGING_EN_ROUTE)
                return "AGV dang sac, khong the ra lenh thu cong luc nay";
            if (agv.state == AGVState::OFFLINE) return "AGV dang OFFLINE, khong the ra lenh";
            if (agv.state == AGVState::AGV_ERROR) return "AGV dang LOI, hay xoa loi (clear_error) truoc";

            auto path = computePathNoLock(agv, targetNode);
            if (path.empty()) return "Khong tim duoc duong tu " + agv.currentNode + " den " + targetNode;

            requeued = requeueWorkNoLock(agv); // không làm mất task đang dở
            assignPath(agv, path, targetNode, AGVState::WORKING);
        }
        if (requeued) emit("AGV_" + agvId + " bi ghi de bang lenh thu cong -> task cu duoc tra ve hang doi");
        notify();
        dispatchPending();
        return "";
    }

    // ---------------- Huỷ task theo đích đến (nút "Huỷ" trên hàng đợi task ở dashboard) ----------------
    // Huỷ MỌI task đang CHỜ trong hàng đợi có cùng targetNode, và nếu có xe nào đang WORKING
    // (hoặc BLOCKED, sắp làm lại) hướng tới đích đó thì cho xe dừng lại (về IDLE) -- KHÔNG trả
    // task đó về hàng đợi (khác với lỗi/mất kết nối, vì đây là người dùng CHỦ ĐỘNG huỷ).
    // Trả về tổng số mục bị ảnh hưởng (task trong hàng đợi + xe bị dừng); 0 nếu không có gì để huỷ.
    int cancelTask(const std::string& targetNode) {
        int removedFromQueue = 0;
        std::vector<std::string> interruptedAgvs;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            std::priority_queue<Task, std::vector<Task>, TaskCmp> remaining;
            while (!tasks_.empty()) {
                Task t = tasks_.top(); tasks_.pop();
                if (t.targetNode == targetNode) removedFromQueue++;
                else remaining.push(t);
            }
            tasks_ = std::move(remaining);

            for (auto& kv : fleet_) {
                AGV& a = kv.second;
                bool goingThere = a.finalTarget == targetNode &&
                    (a.state == AGVState::WORKING ||
                     (a.state == AGVState::BLOCKED && a.blockedResume == AGVState::WORKING));
                if (!goingThere) continue;
                a.activePath.clear();
                a.finalTarget.clear();
                a.blockedResume = AGVState::IDLE;
                a.state = AGVState::IDLE;
                sendCommand(a, "WAIT"); // huỷ kế hoạch cũ, KHÔNG addTaskInternal lại (khác requeueWorkNoLock)
                interruptedAgvs.push_back(a.id);
            }
        }
        for (auto& id : interruptedAgvs) emit("Task toi " + targetNode + " bi nguoi dung huy -> AGV_" + id + " dung lai");
        if (removedFromQueue > 0) emit("Da huy " + std::to_string(removedFromQueue) + " task dang cho trong hang doi toi " + targetNode);
        if (removedFromQueue > 0 || !interruptedAgvs.empty()) { notify(); dispatchPending(); }
        return removedFromQueue + (int)interruptedAgvs.size();
    }
    //  - phát hiện xe mất kết nối
    //  - gửi lại lệnh chưa được ACK / coi là mất liên lạc lệnh nếu quá số lần thử
    //  - cho xe BLOCKED thử lại
    //  - giao task đang chờ cho xe rảnh gần nhất
    //  - kiểm tra pin toàn đội xe
    void tick() {
        checkTimeouts();
        checkAcks();
        resumeBlocked();
        dispatchPending();
        checkBatteryAll();
    }

    json toJson() {
        std::lock_guard<std::mutex> lk(mtx_);
        json j;
        json agvsJ = json::array();
        const auto now = Clock::now();
        for (auto& kv : fleet_) {
            const AGV& a = kv.second;
            long long silentSec = std::chrono::duration_cast<std::chrono::seconds>(now - a.lastSeen).count();
            agvsJ.push_back({
                {"id", a.id}, {"node", a.currentNode}, {"battery", a.battery},
                {"state", toString(a.state)}, {"path", a.activePath},
                {"target", a.finalTarget}, {"priority", a.isPriorityVehicle},
                {"last_seen_s", silentSec}
            });
        }
        j["agvs"] = agvsJ;
        json tasksJ = json::array();
        for (auto& t : taskQueueSnapshotNoLock())
            tasksJ.push_back({{"target", t.targetNode}, {"priority", t.priority}});
        j["pending_tasks"] = tasksJ;
        j["charge_reservations"] = chargeReservation_; // station -> agvId đang giữ/đặt chỗ
        return j;
    }

private:
    struct TaskCmp {
        bool operator()(const Task& a, const Task& b) const {
            if (a.priority != b.priority) return a.priority < b.priority; // priority cao hơn lên top
            return a.seq > b.seq; // seq nhỏ hơn (tạo trước) lên top -> FIFO cùng priority
        }
    };

    static constexpr int kRequeuePriority = 5; // task bị trả lại được ưu tiên hơn task thường

    // 1 lệnh đang chờ ESP32 xác nhận (ACK) — xem [5] trong ghi chú đầu file.
    struct PendingCmd {
        long cmdId;
        std::string topic;
        std::string payload; // gửi lại NGUYÊN VĂN chuỗi này nếu hết ackTimeout_
        Clock::time_point sentAt;
        int retries = 0;
    };

    void addTaskInternal(const std::string& targetNode, int priority) {
        tasks_.push(Task{targetNode, priority, seqCounter_++});
    }

    // PHẢI gọi khi đã giữ mtx_
    std::vector<Task> taskQueueSnapshotNoLock() {
        auto copy = tasks_;
        std::vector<Task> out;
        while (!copy.empty()) { out.push_back(copy.top()); copy.pop(); }
        return out;
    }

    // ---------------- Các hàm phụ (PHẢI gọi khi đã giữ mtx_) ----------------

    // Xe đang có "việc dở" cần được giao lại nếu xe bị loại khỏi cuộc chơi?
    static bool hasWork(const AGV& a) {
        if (a.finalTarget.empty()) return false;
        return a.state == AGVState::WORKING ||
               (a.state == AGVState::BLOCKED && a.blockedResume == AGVState::WORKING);
    }

    // Trả task đang dở (nếu có) về hàng đợi chung, rồi xoá kế hoạch của xe.
    // Trả về true nếu có task được trả lại.
    bool requeueWorkNoLock(AGV& a) {
        bool requeued = false;
        if (hasWork(a)) {
            addTaskInternal(a.finalTarget, kRequeuePriority);
            requeued = true;
        }
        releaseStationReservationNoLock(a.id); // huỷ mọi kế hoạch (kể cả kế hoạch đi sạc) -> nhả chỗ trạm
        a.activePath.clear();
        a.finalTarget.clear();
        a.blockedResume = AGVState::IDLE;
        return requeued;
    }

    // Trạm sạc đang được xe id đặt chỗ hoặc chiếm giữ -> gỡ hết (dùng khi kế hoạch/việc sạc bị huỷ)
    void releaseStationReservationNoLock(const std::string& id) {
        for (auto it = chargeReservation_.begin(); it != chargeReservation_.end(); ) {
            if (it->second == id) it = chargeReservation_.erase(it); else ++it;
        }
    }

    bool isChargingStationNode(const std::string& node) const {
        auto stations = map_.chargingStations();
        return std::find(stations.begin(), stations.end(), node) != stations.end();
    }

    // Chọn trạm sạc gần nhất còn TRỐNG (chưa bị xe khác đặt chỗ/chiếm) mà agv tới được.
    // Khác map_.nearestCharging(): loại các trạm đã có agv KHÁC đặt chỗ, để 2 xe pin thấp
    // không cùng lúc nhắm tới 1 trạm còn đang trống (trước khi ai đó thực sự tới nơi).
    std::string pickChargingStationNoLock(const AGV& agv) {
        std::string best; size_t bestLen = SIZE_MAX;
        for (auto& cs : map_.chargingStations()) {
            auto rit = chargeReservation_.find(cs);
            if (rit != chargeReservation_.end() && rit->second != agv.id) continue; // trạm đã có chủ khác
            auto path = computePathNoLock(agv, cs);
            if (!path.empty() && path.size() < bestLen) { bestLen = path.size(); best = cs; }
        }
        return best;
    }

    // Các node "bận" với xe excludeId: đường đi của xe khác + vị trí đang đứng của xe khác
    // (xe IDLE / đang sạc / lỗi / offline vẫn chiếm chỗ thật ngoài đời).
    std::unordered_set<std::string> buildBusyNodesNoLock(const std::string& excludeId) const {
        std::unordered_set<std::string> busy;
        for (auto& kv : fleet_) {
            if (kv.first == excludeId) continue;
            const AGV& o = kv.second;
            if (!o.currentNode.empty()) busy.insert(o.currentNode);
            for (auto& n : o.activePath) busy.insert(n);
        }
        return busy;
    }

    // Tìm đường: ưu tiên né node bận; nếu bị chặn hết thì fallback đi xuyên (xếp hàng qua mutex zone)
    std::vector<std::string> computePathNoLock(const AGV& agv, const std::string& target) {
        auto busy = buildBusyNodesNoLock(agv.id);
        auto path = map_.findShortestPath(agv.currentNode, target, busy);
        if (path.empty()) path = map_.findShortestPath(agv.currentNode, target);
        return path;
    }

    std::vector<std::string> agvsRoutedThroughNoLock(const std::string& node, const std::string& exceptId) const {
        std::vector<std::string> out;
        if (node.empty()) return out;
        for (auto& kv : fleet_) {
            if (kv.first == exceptId) continue;
            const auto& p = kv.second.activePath;
            if (std::find(p.begin(), p.end(), node) != p.end()) out.push_back(kv.first);
        }
        return out;
    }

    // Dùng resumeBattery_ (không phải lowBattery_) để tránh xe pin 21% nhận task rồi
    // vài phút sau bị huỷ để đi sạc — xem hysteresis ở [6] trong ghi chú đầu file.
    bool dispatchableNoLock(const AGV& a) const {
        return a.state == AGVState::IDLE && a.battery >= resumeBattery_ && !a.currentNode.empty();
    }

    // Tìm & giao AGV IDLE gần nhất (theo số node) cho các task đang chờ,
    // ưu tiên xử lý task có priority cao trước (và tạo trước, nếu bằng priority).
    void dispatchPending() {
        bool assignedAny = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (tasks_.empty()) return;
            bool anyFree = false;
            for (auto& kv : fleet_) if (dispatchableNoLock(kv.second)) { anyFree = true; break; }
            if (!anyFree) return; // không có xe rảnh -> khỏi tốn Dijkstra

            std::priority_queue<Task, std::vector<Task>, TaskCmp> remaining;
            while (!tasks_.empty()) {
                Task t = tasks_.top(); tasks_.pop();

                std::string bestId; std::vector<std::string> bestPath; size_t bestLen = SIZE_MAX;
                // pass 0: né node bận (xe khác đang đứng / sắp đi qua). pass 1: không né.
                for (int pass = 0; pass < 2 && bestId.empty(); ++pass) {
                    for (auto& kv : fleet_) {
                        const AGV& agv = kv.second;
                        if (!dispatchableNoLock(agv)) continue;
                        std::vector<std::string> path;
                        if (pass == 0) path = map_.findShortestPath(agv.currentNode, t.targetNode, buildBusyNodesNoLock(kv.first));
                        else           path = map_.findShortestPath(agv.currentNode, t.targetNode);
                        if (!path.empty() && path.size() < bestLen) {
                            bestLen = path.size();
                            bestId = kv.first;
                            bestPath = path;
                        }
                    }
                }

                if (!bestId.empty()) {
                    assignPath(fleet_[bestId], bestPath, t.targetNode, AGVState::WORKING);
                    assignedAny = true;
                } else {
                    remaining.push(t); // chưa có xe phù hợp -> giữ lại cho lần tick sau
                }
            }
            tasks_ = std::move(remaining);
        }
        if (assignedAny) notify();
    }

    void assignPath(AGV& agv, std::vector<std::string> path, const std::string& target, AGVState newState) {
        if (!path.empty() && path.front() == agv.currentNode) path.erase(path.begin());
        agv.activePath = path;
        agv.finalTarget = target;
        agv.state = newState;
        agv.blockedResume = AGVState::IDLE;
        sendCommand(agv, "GOTO", {{"path", path}, {"target", target}});
    }

    // Gửi 1 lệnh JSON có cmd_id xuống xe, theo dõi để gửi lại nếu không có ACK (xem [5]).
    // PHẢI gọi khi đang giữ mtx_. Lệnh mới cho cùng 1 xe thay thế hẳn lệnh cũ đang chờ ACK.
    void sendCommand(AGV& agv, const std::string& cmdName, json extra = json::object()) {
        long cmdId = ++cmdSeq_;
        json j = extra;
        j["cmd"] = cmdName;
        j["cmd_id"] = cmdId;
        std::string topic = "agv/" + agv.id + "/command";
        std::string payload = j.dump();
        pendingCmd_[agv.id] = PendingCmd{cmdId, topic, payload, Clock::now(), 0};
        if (publish_) publish_(topic, payload);
    }

    // Gửi lại lệnh chưa được ACK sau ackTimeout_; sau maxRetries_ lần mà vẫn im lặng
    // thì coi là mất liên lạc lệnh: trả task về hàng đợi và chuyển xe sang AGV_ERROR.
    void checkAcks() {
        std::vector<std::pair<std::string, bool>> failed; // {agvId, co_requeue_task_khong}
        {
            std::lock_guard<std::mutex> lk(mtx_);
            const auto now = Clock::now();
            for (auto it = pendingCmd_.begin(); it != pendingCmd_.end(); ) {
                const std::string& id = it->first;
                PendingCmd& pc = it->second;
                if (now - pc.sentAt < ackTimeout_) { ++it; continue; }

                auto ait = fleet_.find(id);
                if (ait == fleet_.end() || ait->second.state == AGVState::OFFLINE) {
                    it = pendingCmd_.erase(it); // xe đã mất kết nối/bị xoá -> heartbeat đã/sẽ xử lý riêng
                    continue;
                }
                if (pc.retries < maxRetries_) {
                    pc.retries++;
                    pc.sentAt = now;
                    if (publish_) publish_(pc.topic, pc.payload); // gửi lại NGUYÊN VĂN, cùng cmd_id
                    ++it;
                } else {
                    AGV& agv = ait->second;
                    bool requeued = requeueWorkNoLock(agv);
                    agv.state = AGVState::AGV_ERROR;
                    failed.push_back({id, requeued});
                    it = pendingCmd_.erase(it);
                }
            }
        }
        for (auto& f : failed)
            emit("AGV_" + f.first + " KHONG ACK lenh sau " + std::to_string(maxRetries_) +
                 " lan gui lai -> nghi mat lien lac, chuyen LOI" + (f.second ? " (task da tra ve hang doi)" : ""));
        if (!failed.empty()) { notify(); dispatchPending(); }
    }

    // Tính lại đường cho 1 xe — gọi khi phát hiện vật cản, lệch đường hoặc đường bị chặn
    void replanAGV(const std::string& id) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = fleet_.find(id);
        if (it == fleet_.end() || it->second.finalTarget.empty()) return;
        AGV& agv = it->second;
        if (agv.state != AGVState::WORKING && agv.state != AGVState::CHARGING_EN_ROUTE &&
            agv.state != AGVState::BLOCKED) return;

        AGVState resume = (agv.state == AGVState::BLOCKED) ? agv.blockedResume
                        : (agv.state == AGVState::CHARGING_EN_ROUTE ? AGVState::CHARGING_EN_ROUTE : AGVState::WORKING);
        if (resume != AGVState::WORKING && resume != AGVState::CHARGING_EN_ROUTE) resume = AGVState::WORKING;

        auto path = computePathNoLock(agv, agv.finalTarget);
        if (path.empty()) {
            if (agv.state != AGVState::BLOCKED) {
                agv.blockedResume = resume;
                agv.state = AGVState::BLOCKED; // hết đường -> chờ, tick() sẽ thử lại (resumeBlocked)
                agv.activePath.clear();
                sendCommand(agv, "WAIT");
            }
        } else {
            assignPath(agv, path, agv.finalTarget, resume);
        }
    }

    // Phát hiện xe mất kết nối
    void checkTimeouts() {
        std::vector<std::string> lost, requeuedIds, toReplan;
        bool release = true;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            const auto now = Clock::now();
            for (auto& kv : fleet_) {
                AGV& a = kv.second;
                if (a.state == AGVState::OFFLINE) continue;
                if (now - a.lastSeen <= offlineTimeout_) continue;
                if (requeueWorkNoLock(a)) requeuedIds.push_back(kv.first);
                a.state = AGVState::OFFLINE;
                lost.push_back(kv.first);
                pendingCmd_.erase(kv.first); // đừng phí công gửi lại lệnh cho xe đã coi là mất kết nối
            }
            for (auto& id : lost)
                for (auto& o : agvsRoutedThroughNoLock(fleet_[id].currentNode, id)) toReplan.push_back(o);
            release = releaseZonesOnOffline_;
        }
        if (lost.empty()) return;

        std::sort(toReplan.begin(), toReplan.end());
        toReplan.erase(std::unique(toReplan.begin(), toReplan.end()), toReplan.end());
        for (auto& id : lost) {
            bool requeued = std::find(requeuedIds.begin(), requeuedIds.end(), id) != requeuedIds.end();
            emit("AGV_" + id + " MAT KET NOI (OFFLINE)" + (requeued ? " -> task duoc tra ve hang doi" : ""));
            if (release) releaseAgvZones(id);
        }
        for (auto& o : toReplan) replanAGV(o);
        notify();
        dispatchPending();
    }

    // Cho các xe BLOCKED thử lại mỗi tick
    void resumeBlocked() {
        bool changed = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto& kv : fleet_) {
                AGV& agv = kv.second;
                if (agv.state != AGVState::BLOCKED) continue;
                if (agv.finalTarget.empty()) {
                    // Không còn việc dở. Nếu pin thấp thì để checkBatteryOne tìm đường tới trạm sạc.
                    if (agv.battery > lowBattery_) {
                        agv.state = AGVState::IDLE;
                        agv.blockedResume = AGVState::IDLE;
                        changed = true;
                    }
                    continue;
                }
                auto path = computePathNoLock(agv, agv.finalTarget);
                if (path.empty()) continue; // vẫn kẹt, thử lại tick sau (không gửi lại WAIT)
                AGVState resume = (agv.blockedResume == AGVState::CHARGING_EN_ROUTE)
                                  ? AGVState::CHARGING_EN_ROUTE : AGVState::WORKING;
                assignPath(agv, path, agv.finalTarget, resume);
                changed = true;
            }
        }
        if (changed) notify();
    }

    // Trả về true nếu có thay đổi state/hàng đợi
    bool checkBatteryOne(const std::string& id) {
        bool changed = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = fleet_.find(id);
            if (it == fleet_.end()) return false;
            AGV& agv = it->second;
            if (agv.state == AGVState::OFFLINE || agv.state == AGVState::AGV_ERROR ||
                agv.state == AGVState::CHARGING_EN_ROUTE || agv.state == AGVState::CHARGING) return false;
            if (agv.battery > lowBattery_ || agv.currentNode.empty()) return false;

            // Nếu đang làm task dở -> trả ngay vào hàng đợi chung, ưu tiên cao,
            // để XE KHÁC nhận và tiếp tục ngay (không đợi xe này sạc xong).
            // (Cũng nhả mọi chỗ trạm sạc cũ đang giữ, để pickChargingStationNoLock bên dưới sạch sẽ.)
            bool requeued = requeueWorkNoLock(agv);
            changed = requeued;

            if (isChargingStationNode(agv.currentNode)) {
                // Đã đứng sẵn tại 1 trạm sạc -> đặt chỗ cho chính mình rồi sạc luôn
                chargeReservation_[agv.currentNode] = agv.id;
                agv.state = AGVState::CHARGING;
                sendCommand(agv, "CHARGE_START");
                changed = true;
            } else {
                // Chọn trạm gần nhất CHƯA bị xe khác đặt chỗ (xem pickChargingStationNoLock)
                std::string chg = pickChargingStationNoLock(agv);
                std::vector<std::string> path;
                if (!chg.empty()) path = computePathNoLock(agv, chg);
                if (path.empty()) {
                    if (agv.state != AGVState::BLOCKED) { // không có đường tới trạm sạc nào (trống hoặc đặt được)
                        agv.state = AGVState::BLOCKED;
                        agv.blockedResume = AGVState::IDLE;
                        changed = true;
                    }
                } else {
                    chargeReservation_[chg] = agv.id; // đặt chỗ NGAY khi cử đi, trước khi tới nơi
                    assignPath(agv, path, chg, AGVState::CHARGING_EN_ROUTE);
                    changed = true;
                }
            }
        }
        if (changed) {
            notify();
            dispatchPending(); // có thể có xe khác rảnh để nhận ngay task vừa trả lại
        }
        return changed;
    }

    void checkBatteryAll() {
        std::vector<std::string> ids;
        { std::lock_guard<std::mutex> lk(mtx_); for (auto& kv : fleet_) ids.push_back(kv.first); }
        for (auto& id : ids) checkBatteryOne(id);
    }

    // Nhả mọi zone xe đang giữ và báo "mutex_grant" cho xe kế tiếp đang xếp hàng ở từng zone.
    // (Cần MutexArbiter::forceRelease bản mới, trả về danh sách {zone, xe_được_cấp}.)
    // KHÔNG gọi khi đang giữ mtx_.
    void releaseAgvZones(const std::string& id) {
        auto grants = arbiter_.forceRelease(id);
        if (!publish_) return;
        for (auto& g : grants) {
            publish_("agv/" + g.second + "/mutex_grant", g.first);
            emit("Zone " + g.first + " duoc cap cho AGV_" + g.second + " (xe " + id + " roi khoi he thong)");
        }
    }

    void notify() { if (onChanged_) onChanged_(); }
    void emit(const std::string& msg) { if (onEvent_) onEvent_(msg); }

    MapGraph& map_;
    MutexArbiter& arbiter_;
    std::unordered_map<std::string, AGV> fleet_;
    std::priority_queue<Task, std::vector<Task>, TaskCmp> tasks_;
    long seqCounter_ = 0;
    int lowBattery_ = 20;    // % pin coi là "thấp" -> tự động về sạc
    int resumeBattery_ = 40; // % pin tối thiểu để được nhận task mới (hysteresis, phải > lowBattery_)
    int chargeTargetPercent_ = 90; // % pin coi là "sạc xong" nếu ESP32 không báo kèm số liệu thật
    std::chrono::seconds offlineTimeout_{10};
    bool autoRegister_ = true;
    bool releaseZonesOnOffline_ = true;
    std::unordered_map<std::string, std::string> chargeReservation_; // station -> agvId đang giữ/đặt chỗ
    std::unordered_map<std::string, PendingCmd> pendingCmd_;         // agvId -> lệnh đang chờ ACK
    long cmdSeq_ = 0;
    std::chrono::milliseconds ackTimeout_{3000};
    int maxRetries_ = 3;
    std::function<void(const std::string&, const std::string&)> publish_;
    std::function<void()> onChanged_;
    std::function<void(const std::string&)> onEvent_;
    std::mutex mtx_;
};