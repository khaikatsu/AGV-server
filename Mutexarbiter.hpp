#pragma once
// ============================================================================
// FILE 2/4 — MutexArbiter.hpp
// Phụ trách: NGƯỜI 2 — Trọng tài khoá chéo (Mutex Zone Arbiter)
// Vai trò:
//   - Đảm bảo tại một thời điểm chỉ 1 AGV được đi vào 1 ngã tư (mutex zone)
//     để tránh va chạm giữa nhiều xe chạy trên nhiều board (ESP32) khác nhau,
//     giao tiếp qua MQTT (không dùng std::mutex thường được vì mỗi xe là
//     1 tiến trình/thiết bị vật lý riêng biệt).
//   - Thread-safe để dùng chung an toàn giữa luồng MQTT (khi request/release)
//     và luồng WebSocket (khi hiển thị trạng thái lên dashboard).
//
// Cách dùng (do Người 4 - Server - gọi trong on_message của MQTT):
//   - requestZone(zone, agvId) -> true: được cấp quyền ngay, publish "grant".
//                                 false: đã xếp hàng chờ, không publish gì.
//   - releaseZone(zone, agvId) -> nếu có xe kế tiếp trong hàng đợi, trả về
//     ID xe đó (đã được cấp quyền) để server publish "grant" báo xe đó chạy.
// ============================================================================
#include <string>
#include <vector>
#include <unordered_map>
#include <deque>
#include <mutex>
#include <algorithm>
#include <utility>
#include "json.hpp"

using json = nlohmann::json;

class MutexArbiter {
public:
    explicit MutexArbiter(const std::vector<std::string>& zoneIds = {}) {
        for (const auto& id : zoneIds) owner_[id] = "";
    }

    // Đăng ký thêm 1 zone mới lúc runtime (khi map được chỉnh sửa từ web)
    void registerZone(const std::string& zoneId) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!owner_.count(zoneId)) owner_[zoneId] = "";
    }

    // Xe agvId muốn vào zoneId.
    // true = được cấp quyền ngay lập tức. false = phải chờ (đã xếp hàng đợi).
    bool requestZone(const std::string& zoneId, const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = owner_.find(zoneId);
        if (it == owner_.end()) { owner_[zoneId] = agvId; return true; } // zone chưa có -> cấp luôn
        if (it->second.empty()) { it->second = agvId; return true; }
        if (it->second == agvId) return true; // xe này đang giữ zone rồi

        auto& q = queue_[zoneId];
        for (const auto& w : q) if (w == agvId) return false; // đã xếp hàng sẵn, tránh trùng
        q.push_back(agvId);
        return false;
    }

    // Xe agvId rời khỏi zoneId.
    // Trả về ID xe kế tiếp (đã được cấp quyền) nếu có ai đang chờ, ngược lại "".
    std::string releaseZone(const std::string& zoneId, const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = owner_.find(zoneId);
        if (it == owner_.end() || it->second != agvId) return ""; // không phải chủ sở hữu -> bỏ qua

        auto qit = queue_.find(zoneId);
        if (qit != queue_.end() && !qit->second.empty()) {
            std::string next = qit->second.front();
            qit->second.pop_front();
            it->second = next;
            return next;
        }
        it->second.clear();
        return "";
    }

    std::string currentOwner(const std::string& zoneId) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = owner_.find(zoneId);
        return it == owner_.end() ? "" : it->second;
    }

    // Gỡ AGV khỏi mọi zone + mọi hàng đợi (khi xe bị xoá khỏi hệ thống, mất kết nối hoặc lỗi nặng).
    // Với mỗi zone xe này đang giữ mà CÓ xe đang xếp hàng, quyền được trao luôn cho xe đầu hàng.
    // Trả về danh sách {zoneId, xe_mới_được_cấp} để Server publish "mutex_grant" cho các xe đó
    // (nếu không, xe đang chờ sẽ đứng mãi vì không ai báo cho nó biết zone đã trống).
    // Nơi gọi cũ không dùng giá trị trả về vẫn biên dịch bình thường.
    std::vector<std::pair<std::string, std::string>> forceRelease(const std::string& agvId) {
        std::lock_guard<std::mutex> lk(mtx_);
        std::vector<std::pair<std::string, std::string>> grants;
        // Bỏ xe này khỏi mọi hàng đợi trước, để nó không bị "cấp quyền cho chính mình"
        for (auto& kv : queue_) kv.second.erase(std::remove(kv.second.begin(), kv.second.end(), agvId), kv.second.end());
        for (auto& kv : owner_) {
            if (kv.second != agvId) continue;
            auto qit = queue_.find(kv.first);
            if (qit != queue_.end() && !qit->second.empty()) {
                kv.second = qit->second.front();
                qit->second.pop_front();
                grants.push_back({kv.first, kv.second});
            } else {
                kv.second.clear();
            }
        }
        return grants;
    }

    json toJson() {
        std::lock_guard<std::mutex> lk(mtx_);
        json j;
        for (auto& kv : owner_) {
            json z;
            z["owner"] = kv.second;
            json waiters = json::array();
            auto qit = queue_.find(kv.first);
            if (qit != queue_.end()) for (auto& w : qit->second) waiters.push_back(w);
            z["waiting"] = waiters;
            j[kv.first] = z;
        }
        return j;
    }

private:
    std::unordered_map<std::string, std::string> owner_;             // zoneId -> agvId đang giữ ("" = trống)
    std::unordered_map<std::string, std::deque<std::string>> queue_; // zoneId -> danh sách agvId đang chờ
    std::mutex mtx_;
};