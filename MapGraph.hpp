#pragma once
// ============================================================================
// FILE 1/4 — MapGraph.hpp
// Phụ trách: NGƯỜI 1 — Bản đồ & thuật toán tìm đường
// Vai trò:
//   - Cấu trúc dữ liệu bản đồ (Node/Edge), nạp & lưu ra file JSON.
//   - Cho phép CHỈNH SỬA MAP LÚC RUNTIME (thêm/xoá node, cạnh) để phục vụ
//     tính năng "điều chỉnh map" từ dashboard web.
//   - Đánh dấu & gỡ vật cản (obstacle) theo từng node — phục vụ né vật cản.
//   - Dijkstra tìm đường ngắn nhất, có thể né tạm các node "đang bận"
//     (do AGV khác đang đi qua) MÀ KHÔNG sửa đổi trạng thái chung của đồ thị
//     (khác bản gốc: bản gốc dùng blockNodes()/clearBlockages() làm bẩn state
//     toàn cục -> nếu 2 luồng cùng gọi dispatch cùng lúc sẽ đá nhau / sai kết
//     quả. Bản này truyền tập node cần né dưới dạng tham số cục bộ).
//   - Thread-safe bằng std::mutex vì Web UI (luồng WebSocket) và Dispatcher
//     (luồng MQTT / tick định kỳ) có thể động vào map cùng lúc.
//
// Ghi chú build: cần thư viện header-only nlohmann/json (json.hpp) — tải tại
// https://github.com/nlohmann/json (file single_include/nlohmann/json.hpp),
// đặt cùng thư mục và đổi tên thành "json.hpp".
// ============================================================================
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <mutex>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <climits>
#include "json.hpp"

using json = nlohmann::json;

struct Node {
    std::string id;
    double x = 0.0, y = 0.0;
    std::string type = "waypoint"; // waypoint | mutex | charging | station
    bool isMutex() const { return type == "mutex"; }
    bool isCharging() const { return type == "charging"; }
};

struct Edge {
    std::string from;
    std::string to;
    int weight = 1;
    std::string type = "main";  // main | branch | loop
    std::string path_class;     // boulevard | shortcut | loop | branch ...
    bool twoWay = false;        // true = nhánh chữ T 2 chiều (yêu cầu nhường đường)
};

class MapGraph {
public:
    // ---------------- Nạp / lưu file JSON ----------------
    void loadFromFile(const std::string& path) {
        std::ifstream in(path);
        if (!in.is_open()) throw std::runtime_error("Khong mo duoc file map: " + path);
        json j; in >> j;
        loadFromJson(j);
    }

    // Nạp bản đồ trực tiếp từ 1 đối tượng JSON đã có sẵn trong bộ nhớ (không qua file) --
    // dùng khi Web Dashboard gửi lệnh "load_map_data" (nạp lại 1 snapshot đã lưu trong SQLite,
    // hoặc nạp bản đồ mới nhất khi Node.js gateway vừa kết nối tới C++ Core). Cùng định dạng
    // JSON với toJson()/loadFromFile(), nên nạp lại đúng những gì đã lưu, bao gồm cả obstacles_
    // (bản gốc loadFromFile() vô tình BỎ QUÊN không khôi phục obstacles khi nạp file).
    void loadFromJson(const json& j) {
        std::lock_guard<std::mutex> lk(mtx_);
        clearInternal();
        mapName_ = j.value("map_name", "");

        for (const auto& n : j.at("nodes")) {
            Node node;
            node.id   = n.at("id").get<std::string>();
            node.x    = n.value("x", 0.0);
            node.y    = n.value("y", 0.0);
            node.type = n.value("type", "waypoint");
            nodes_[node.id] = node;
            if (node.isMutex())    addUnique(mutexZones_, node.id);
            if (node.isCharging()) addUnique(chargingStations_, node.id);
        }
        for (const auto& e : j.at("edges")) {
            Edge edge;
            edge.from       = e.at("from").get<std::string>();
            edge.to         = e.at("to").get<std::string>();
            edge.weight     = e.value("weight", 1);
            edge.type       = e.value("type", "main");
            edge.path_class = e.value("path_class", "");
            edge.twoWay     = e.value("two_way", false);
            addEdgeInternal(edge);
        }
        for (const auto& id : j.value("mutex_zones", std::vector<std::string>{})) addUnique(mutexZones_, id);
        for (const auto& id : j.value("charging_stations", std::vector<std::string>{})) addUnique(chargingStations_, id);
        for (const auto& id : j.value("obstacles", std::vector<std::string>{})) obstacles_.insert(id);
    }

    void saveToFile(const std::string& path) {
        std::lock_guard<std::mutex> lk(mtx_);
        json j;
        j["map_name"] = mapName_;
        for (auto& kv : nodes_) {
            const Node& n = kv.second;
            j["nodes"].push_back({{"id", n.id}, {"x", n.x}, {"y", n.y}, {"type", n.type}});
        }
        for (auto& e : edges_) {
            j["edges"].push_back({{"from", e.from}, {"to", e.to}, {"weight", e.weight},
                                   {"type", e.type}, {"path_class", e.path_class}, {"two_way", e.twoWay}});
        }
        j["mutex_zones"] = mutexZones_;
        j["charging_stations"] = chargingStations_;
        std::ofstream out(path);
        out << j.dump(2);
    }

    // ---------------- Chỉnh sửa map lúc runtime (dùng cho Web UI) ----------------
    void addNode(const std::string& id, double x, double y, const std::string& type) {
        std::lock_guard<std::mutex> lk(mtx_);
        nodes_[id] = Node{id, x, y, type};
        if (type == "mutex")    addUnique(mutexZones_, id);
        if (type == "charging") addUnique(chargingStations_, id);
    }

    void removeNode(const std::string& id) {
        std::lock_guard<std::mutex> lk(mtx_);
        nodes_.erase(id);
        adjacency_.erase(id);
        for (auto& kv : adjacency_) {
            auto& list = kv.second;
            list.erase(std::remove_if(list.begin(), list.end(),
                       [&](const Edge& e){ return e.to == id; }), list.end());
        }
        edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
                     [&](const Edge& e){ return e.from == id || e.to == id; }), edges_.end());
        obstacles_.erase(id);
    }

    void addEdge(const std::string& from, const std::string& to, int weight,
                 const std::string& type = "main", const std::string& pathClass = "",
                 bool twoWay = false) {
        std::lock_guard<std::mutex> lk(mtx_);
        addEdgeInternal(Edge{from, to, weight, type, pathClass, twoWay});
        if (twoWay) addEdgeInternal(Edge{to, from, weight, type, pathClass, twoWay});
    }

    void removeEdge(const std::string& from, const std::string& to) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = adjacency_.find(from);
        if (it != adjacency_.end())
            it->second.erase(std::remove_if(it->second.begin(), it->second.end(),
                              [&](const Edge& e){ return e.to == to; }), it->second.end());
        edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
                     [&](const Edge& e){ return e.from == from && e.to == to; }), edges_.end());
    }

    // Vật cản: cảm biến AGV báo vật cản tại 1 node -> chặn tạm thời cho Dijkstra
    void setObstacle(const std::string& nodeId, bool blocked) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (blocked) obstacles_.insert(nodeId); else obstacles_.erase(nodeId);
    }
    bool hasObstacle(const std::string& nodeId) {
        std::lock_guard<std::mutex> lk(mtx_);
        return obstacles_.count(nodeId) > 0;
    }

    // ---------------- Dijkstra ----------------
    // extraBlocked: các node cần né TẠM cho riêng lần gọi này (VD: đường đi
    // hiện tại của các AGV khác) — không sửa state chung -> an toàn đa luồng.
    std::vector<std::string> findShortestPath(const std::string& start, const std::string& target,
                                               const std::unordered_set<std::string>& extraBlocked = {}) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!nodes_.count(start) || !nodes_.count(target)) return {};

        std::unordered_map<std::string, int> dist;
        std::unordered_map<std::string, std::string> prev;
        for (auto& kv : nodes_) dist[kv.first] = INT_MAX;
        dist[start] = 0;

        using PQItem = std::pair<int, std::string>;
        std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
        pq.push({0, start});

        while (!pq.empty()) {
            int d = pq.top().first; std::string u = pq.top().second; pq.pop();
            if (u == target) break;
            if (d > dist[u]) continue;
            auto it = adjacency_.find(u);
            if (it == adjacency_.end()) continue;
            for (const auto& edge : it->second) {
                if (obstacles_.count(edge.to) || extraBlocked.count(edge.to)) continue;
                int alt = dist[u] + edge.weight;
                if (dist.find(edge.to) != dist.end() && alt < dist[edge.to]) {
                    dist[edge.to] = alt;
                    prev[edge.to] = u;
                    pq.push({alt, edge.to});
                }
            }
        }
        std::vector<std::string> path;
        if (start != target && (dist.find(target) == dist.end() || dist[target] == INT_MAX)) return path;
        std::string cur = target;
        while (prev.count(cur)) { path.insert(path.begin(), cur); cur = prev[cur]; }
        path.insert(path.begin(), start);
        return path;
    }

    std::string nearestCharging(const std::string& from, const std::unordered_set<std::string>& extraBlocked = {}) {
        std::string best; size_t bestLen = SIZE_MAX;
        for (auto& cs : chargingStations()) {
            auto p = findShortestPath(from, cs, extraBlocked);
            if (!p.empty() && p.size() < bestLen) { bestLen = p.size(); best = cs; }
        }
        return best;
    }

    std::vector<std::string> mutexZones() { std::lock_guard<std::mutex> lk(mtx_); return mutexZones_; }
    std::vector<std::string> chargingStations() { std::lock_guard<std::mutex> lk(mtx_); return chargingStations_; }
    bool nodeExists(const std::string& id) { std::lock_guard<std::mutex> lk(mtx_); return nodes_.count(id) > 0; }

    json toJson() {
        std::lock_guard<std::mutex> lk(mtx_);
        json j;
        j["map_name"] = mapName_;
        for (auto& kv : nodes_) {
            const Node& n = kv.second;
            j["nodes"].push_back({{"id", n.id}, {"x", n.x}, {"y", n.y}, {"type", n.type}});
        }
        for (auto& e : edges_) {
            j["edges"].push_back({{"from", e.from}, {"to", e.to}, {"weight", e.weight},
                                   {"type", e.type}, {"path_class", e.path_class}, {"two_way", e.twoWay}});
        }
        j["obstacles"] = std::vector<std::string>(obstacles_.begin(), obstacles_.end());
        j["mutex_zones"] = mutexZones_;
        j["charging_stations"] = chargingStations_;
        return j;
    }

    // Map demo khop so do thuc te (don vi cm): 4 ngã tư mutex M1..M4, cac GATE
    // trung chuyen (GATE_S1/S2, GATE_K1..K6), 4 goc CORNER, 2 tram sac, 6 diem
    // kho (KHO_1..KHO_6) noi 2 chieu qua GATE. Toa do va trong so canh la
    // khoang cach thuc te (cm) de dieu huong AGV chinh xac theo ty le ngoai doi.
    // Dung khi chua co map_layout.json de he thong chay duoc ngay.
    void loadDefaultDemoMap() {
        std::lock_guard<std::mutex> lk(mtx_);
        clearInternal();
        mapName_ = "Warehouse Layout v2 (ty le thuc te - cm)";

        // Toa do don vi: cm (ty le thuc te ngoai doi)
        addNodeNoLock("CORNER_TL", 0, 0, "waypoint");
        addNodeNoLock("RFID_M1", 50, 0, "mutex");
        addNodeNoLock("GATE_S1", 100, 0, "waypoint");
        addNodeNoLock("RFID_M2", 150, 0, "mutex");
        addNodeNoLock("CORNER_TR", 200, 0, "waypoint");
        addNodeNoLock("CHG_1", 100, -20, "charging");
        addNodeNoLock("CHG_2", 100, 160, "charging");
        addNodeNoLock("GATE_K2", 50, 50, "waypoint");
        addNodeNoLock("GATE_K4", 150, 50, "waypoint");
        addNodeNoLock("KHO_2", 30, 50, "station");
        addNodeNoLock("KHO_4", 170, 50, "station");
        addNodeNoLock("GATE_K1", 0, 70, "waypoint");
        addNodeNoLock("GATE_K6", 200, 70, "waypoint");
        addNodeNoLock("KHO_1", -20, 70, "station");
        addNodeNoLock("KHO_6", 220, 70, "station");
        addNodeNoLock("GATE_K3", 50, 90, "waypoint");
        addNodeNoLock("GATE_K5", 150, 90, "waypoint");
        addNodeNoLock("KHO_3", 30, 90, "station");
        addNodeNoLock("KHO_5", 170, 90, "station");
        addNodeNoLock("CORNER_BL", 0, 140, "waypoint");
        addNodeNoLock("RFID_M3", 50, 140, "mutex");
        addNodeNoLock("GATE_S2", 100, 140, "waypoint");
        addNodeNoLock("RFID_M4", 150, 140, "mutex");
        addNodeNoLock("CORNER_BR", 200, 140, "waypoint");

        // Duong chinh 1 chieu (weight = khoang cach thuc te, don vi cm)
        // Hang tren (trai -> phai)
        addEdgeInternal({"CORNER_TL", "RFID_M1", 50, "main", "boulevard", false});
        addEdgeInternal({"RFID_M1", "GATE_S1", 50, "main", "boulevard", false});
        addEdgeInternal({"GATE_S1", "RFID_M2", 50, "main", "boulevard", false});
        addEdgeInternal({"RFID_M2", "CORNER_TR", 50, "main", "boulevard", false});
        // Hang duoi (phai -> trai)
        addEdgeInternal({"CORNER_BR", "RFID_M4", 50, "main", "boulevard", false});
        addEdgeInternal({"RFID_M4", "GATE_S2", 50, "main", "boulevard", false});
        addEdgeInternal({"GATE_S2", "RFID_M3", 50, "main", "boulevard", false});
        addEdgeInternal({"RFID_M3", "CORNER_BL", 50, "main", "boulevard", false});
        // Cot ngoai trai/phai (tren -> duoi)
        addEdgeInternal({"CORNER_TL", "GATE_K1", 70, "main", "outer", false});
        addEdgeInternal({"GATE_K1", "CORNER_BL", 70, "main", "outer", false});
        addEdgeInternal({"CORNER_TR", "GATE_K6", 70, "main", "outer", false});
        addEdgeInternal({"GATE_K6", "CORNER_BR", 70, "main", "outer", false});
        // Cot trong trai (tren -> duoi)
        addEdgeInternal({"RFID_M1", "GATE_K2", 50, "main", "inner", false});
        addEdgeInternal({"GATE_K2", "GATE_K3", 40, "main", "inner", false});
        addEdgeInternal({"GATE_K3", "RFID_M3", 50, "main", "inner", false});
        // Cot trong phai (duoi -> tren)
        addEdgeInternal({"RFID_M4", "GATE_K5", 50, "main", "inner", false});
        addEdgeInternal({"GATE_K5", "GATE_K4", 40, "main", "inner", false});
        addEdgeInternal({"GATE_K4", "RFID_M2", 50, "main", "inner", false});
        // Duong ngang giua (trai -> phai)
        addEdgeInternal({"GATE_K1", "GATE_K6", 200, "main", "shortcut", false});

        // Nhanh 2 chieu ra diem sac / diem kho (weight = khoang cach thuc te, cm)
        addTwoWay("CHG_1", "GATE_S1", 20, "branch");
        addTwoWay("CHG_2", "GATE_S2", 20, "branch");
        addTwoWay("KHO_1", "GATE_K1", 20, "branch");
        addTwoWay("KHO_2", "GATE_K2", 20, "branch");
        addTwoWay("KHO_3", "GATE_K3", 20, "branch");
        addTwoWay("KHO_4", "GATE_K4", 20, "branch");
        addTwoWay("KHO_5", "GATE_K5", 20, "branch");
        addTwoWay("KHO_6", "GATE_K6", 20, "branch");

        mutexZones_ = {"RFID_M1","RFID_M2","RFID_M3","RFID_M4"};
        chargingStations_ = {"CHG_1", "CHG_2"};
    }
private:
    static void addUnique(std::vector<std::string>& v, const std::string& id) {
        if (std::find(v.begin(), v.end(), id) == v.end()) v.push_back(id);
    }
    void addNodeNoLock(const std::string& id, double x, double y, const std::string& type) {
        nodes_[id] = Node{id, x, y, type};
    }
    void addTwoWay(const std::string& a, const std::string& b, int w, const std::string& type) {
        addEdgeInternal({a, b, w, type, "", true});
        addEdgeInternal({b, a, w, type, "", true});
    }
    void addEdgeInternal(const Edge& e) {
        edges_.push_back(e);
        adjacency_[e.from].push_back(e);
    }
    void clearInternal() {
        nodes_.clear(); edges_.clear(); adjacency_.clear();
        mutexZones_.clear(); chargingStations_.clear(); obstacles_.clear();
    }

    std::string mapName_;
    std::unordered_map<std::string, Node> nodes_;
    std::vector<Edge> edges_;
    std::unordered_map<std::string, std::vector<Edge>> adjacency_;
    std::vector<std::string> mutexZones_;
    std::vector<std::string> chargingStations_;
    std::unordered_set<std::string> obstacles_;
    std::mutex mtx_;
};