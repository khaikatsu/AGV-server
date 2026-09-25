// ============================================================================
// FILE 4/4 — AGVServer.cpp
// Phụ trách: NGƯỜI 4 (mạng MQTT) + QUẢN LÝ (rà soát & ghép nối toàn hệ thống)
//
// File này là "bộ não" chạy chính của server:
//   1. MQTT   — giao tiếp với các xe AGV (ESP32) qua broker Mosquitto.
//   2. WebSocket — phục vụ dashboard web: đẩy trạng thái real-time (map,
//      từng xe, hàng đợi task, trạng thái mutex zone) và nhận lệnh điều
//      khiển từ web (thêm xe, giao task, sửa map, đánh dấu vật cản...).
//   3. Vòng lặp chính: gọi FleetManager::tick() định kỳ để dispatch task
//      đang chờ + kiểm tra pin toàn đội xe.
//
// GHI CHÚ CỦA QUẢN LÝ (những gì đã rà soát & sửa so với bản gốc):
//   - Bản gốc dùng vector<Edge> có cờ isBlocked toàn cục, sửa/dọn liên tục
//     mỗi lần dispatch -> dễ lỗi khi nhiều luồng cùng chạy. Đã tách hẳn
//     việc "né đường" thành tham số cục bộ trong MapGraph::findShortestPath.
//   - Bản gốc chỉ điều khiển thủ công 1 xe bằng phím Enter/Shift -> đã thay
//     bằng dashboard web điều khiển toàn bộ đội xe qua WebSocket.
//   - Đã tách MutexArbiter, MapGraph, FleetManager thành 3 module độc lập,
//     Server chỉ đóng vai trò "dán keo" (glue code) qua dependency injection
//     (setPublishCallback / setStateChangedCallback) — dễ bảo trì, dễ test.
//   - Thêm cơ chế đa luồng an toàn: mỗi module tự khoá mutex nội bộ, Server
//     không giữ khoá xuyên suốt giữa các lệnh gọi.
//
// ---------------------------------------------------------------------------
// GIAO THỨC MQTT (ESP32 <-> Server), topic dạng "agv/{id}/{action}":
//   agv/{id}/telemetry      (ESP32 -> Server)  payload: "RFID|battery" hoặc
//                                               "RFID|battery|obstacle(0/1)"
//   agv/{id}/mutex_request  (ESP32 -> Server)  payload: zoneId
//   agv/{id}/mutex_release  (ESP32 -> Server)  payload: zoneId
//   agv/{id}/mutex_grant    (Server -> ESP32)  payload: zoneId
//   agv/{id}/obstacle       (ESP32 -> Server)  payload: nodeId phát hiện vật cản
//   agv/{id}/error          (ESP32 -> Server)  payload: mô tả lỗi
//   agv/{id}/charging_done  (ESP32 -> Server)  payload: RONG, hoac % pin that luc sac xong
//                                               (VD "87"). Neu rong -> Server dung target mac dinh.
//   agv/{id}/ack            (ESP32 -> Server)  payload: cmd_id (so) cua lenh vua nhan duoc,
//                                               gui ngay khi nhan lenh (khong can doi thuc hien xong)
//   agv/{id}/command        (Server -> ESP32)  payload: LUON la JSON, co "cmd_id":
//                                               {"cmd":"GOTO","cmd_id":42,"path":[...],"target":"..."}
//                                               {"cmd":"STOP","cmd_id":43} / "WAIT" / "CHARGE_START"
//                                               Neu khong thay ack trong vai giay, Server GUI LAI
//                                               nguyen van cung cmd_id (ESP32 bo qua neu da xu ly
//                                               roi nhung van phai ack lai de Server ngung gui lai).
//
// GIAO THỨC WEBSOCKET (Web Dashboard <-> Server), JSON text frame:
//   Server -> Web (định kỳ mỗi ~700ms, hoặc ngay khi có thay đổi):
//     { "map": {...}, "fleet": {...}, "mutex": {...} }
//   Web -> Server (lệnh điều khiển), field "cmd":
//     add_agv {id,start,priority} | remove_agv {id} | add_task {target,priority}
//     direct_goto {agv_id,target}  -- ra lệnh 1 xe CỤ THỂ đi thẳng tới target,
//       bỏ qua bước tự chọn "xe gần nhất" (dùng khi bấm chọn xe rồi bấm đích
//       trực tiếp trên bản đồ ở dashboard)
//     add_node {id,x,y,type} | remove_node {id} | add_edge {from,to,weight,type,path_class,two_way}
//     remove_edge {from,to} | set_obstacle {node,blocked} | save_map {path}
//     clear_error {id}  -- xoa trang thai LOI cua xe sau khi da xu ly (telemetry KHONG tu xoa loi)
//
// ---------------------------------------------------------------------------
// BUILD (Windows, vd. dùng vcpkg):
//   vcpkg install mosquitto nlohmann-json
//   Build Windows      : link ws2_32.lib + OpenSSL (libssl/libcrypto) + mosquitto.lib
//   Build Linux/RaspPi  : g++ -std=c++17 AGVserver.cpp -lssl -lcrypto -lmosquitto -lpthread -o agv_core
//   (SHA1 + Base64 cho bat tay WebSocket dung OpenSSL -- cung 1 bo thu vien tren
//    CA 2 nen tang, khong con phu thuoc rieng CNG/CryptoAPI cua Windows nen chay
//    duoc tren Linux/Raspberry Pi. GCC < 9 can them -lstdc++fs cho <filesystem>.)
// ============================================================================
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <chrono>
#include <deque>
#include <ctime>
#include <cstdint>
#include <csignal>
#include <filesystem>

// ---------------------------------------------------------------------------
// LOP SOCKET CHEO NEN TANG: thay vi chi dung winsock2.h (chi co tren Windows),
// Linux/Raspberry Pi dung thang BSD sockets (POSIX) co san trong glibc, KHONG
// can cai them thu vien socket nao. Dinh nghia lai vai ten (SOCKET,
// INVALID_SOCKET, SOCKET_ERROR, closesocket) de toan bo code WebSocket ben duoi
// (da viet san) dung chung duoc tren ca 2 nen tang ma khong phai sua gi them.
// ---------------------------------------------------------------------------
#if defined(_WIN32)
  #define WIN32_LEAN_AND_MEAN
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #pragma comment(lib, "ws2_32.lib")
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  using SOCKET = int;
  static constexpr SOCKET INVALID_SOCKET = -1;
  static constexpr int SOCKET_ERROR = -1;
  inline int closesocket(SOCKET s) { return ::close(s); }
#endif

// OpenSSL: dung chung cho SHA1 + Base64 cua bat tay WebSocket tren CA 2 nen tang
// (xem base64Encode()/sha1Base64() ben duoi), thay the BCrypt/CryptoAPI cu.
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <mosquitto.h>

#include "json.hpp"
#include "MapGraph.hpp"
#include "MutexArbiter.hpp"
#include "FleetManager.hpp"

using json = nlohmann::json;

// Co Ctrl+C (SIGINT) de thoat chuong trinh sach se tren MOI nen tang, thay the
// GetAsyncKeyState(VK_ESCAPE) (chi hoat dong tren Windows, va chi khi cua so
// console dang duoc focus). Xem vong lap chinh trong main().
static std::atomic<bool> g_running{true};
static void onShutdownSignal(int) { g_running = false; }

// ---------------------------------------------------------------------------
// CO CHE HUONG SU KIEN (EVENT-DRIVEN) cho vong lap chinh: thay vi luon ngu du
// 700ms roi moi tick()/broadcast() (gay tre pha xu ly khi ESP32 vua bao vat
// can/loi qua MQTT), luong MQTT (on_message, chay tren luong rieng cua
// mosquitto_loop_start) goi requestWake() de DANH THUC NGAY vong lap chinh
// trong main() khi co su kien khan cap (obstacle/error) -- xem cac cho goi
// requestWake() trong on_message() ben duoi. 700ms van la "tran tren" (upper
// bound) cho cac lan tick() dinh ky binh thuong khi khong co su kien nao.
// ---------------------------------------------------------------------------
static std::mutex              g_wakeMtx;
static std::condition_variable g_wakeCv;
static std::atomic<bool>       g_wakeRequested{false};

static void requestWake() {
    { std::lock_guard<std::mutex> lk(g_wakeMtx); g_wakeRequested = true; }
    g_wakeCv.notify_one();
}
using namespace std;

// =========================================================================
// PHẦN A — TRẠNG THÁI TOÀN CỤC (glue giữa 3 module)
// =========================================================================
static MapGraph      g_map;
static MutexArbiter  g_arbiter;
static FleetManager  g_fleet(g_map, g_arbiter);
static mosquitto*    g_mosq = nullptr;
// Cổng 3000 dành cho HTTP/WebSocket gateway của Node.js.  C++ Core phải
// dùng cổng riêng để Node.js có thể kết nối đến nó mà không tranh chấp port.
static const int     WS_PORT = 8080;

// -------------------------------------------------------------------------
// NHAT KY SU KIEN HE THONG (moi hanh dong server thuc hien: them/xoa xe, giao
// task, khoa node, luu ban do, loi xe, mat ket noi...). Duoc phat kem trong
// moi lan broadcastState() (truong "logs") de dashboard hien thi thong bao
// thoi gian thuc; Node.js lai luu ben ngoai vao SQLite theo "seq" de tranh
// luu trung khi cung 1 muc duoc gui lai nhieu lan (broadcast lap lai ~700ms).
// -------------------------------------------------------------------------
struct LogEntry { long seq; string time; string msg; };
static mutex        g_logMtx;
static deque<LogEntry> g_logs;      // gioi han LOG_MAX muc gan nhat trong bo nho
static long          g_logSeq = 0;
static const size_t  LOG_MAX = 300; // luu trong RAM; lich su day du nam o SQLite (Node.js)
static const size_t  LOG_BROADCAST = 20; // chi gui N muc gan nhat moi lan de nhe payload

static string nowStr() {
    time_t t = time(nullptr); tm lt{};
#if defined(_WIN32)
    localtime_s(&lt, &t);
#else
    localtime_r(&t, &lt);
#endif
    char buf[16];
    strftime(buf, sizeof(buf), "%H:%M:%S", &lt);
    return string(buf);
}

// Ghi 1 su kien vao nhat ky (thread-safe). Goi tu ca luong MQTT lan luong WebSocket.
static void pushLog(const string& msg) {
    lock_guard<mutex> lk(g_logMtx);
    g_logs.push_back(LogEntry{ ++g_logSeq, nowStr(), msg });
    if (g_logs.size() > LOG_MAX) g_logs.pop_front();
}

static json recentLogsJson() {
    lock_guard<mutex> lk(g_logMtx);
    json arr = json::array();
    size_t start = g_logs.size() > LOG_BROADCAST ? g_logs.size() - LOG_BROADCAST : 0;
    for (size_t i = start; i < g_logs.size(); ++i)
        arr.push_back({ {"seq", g_logs[i].seq}, {"time", g_logs[i].time}, {"msg", g_logs[i].msg} });
    return arr;
}

void publishTo(const string& topic, const string& payload) {
    if (g_mosq) mosquitto_publish(g_mosq, NULL, topic.c_str(), (int)payload.size(), payload.c_str(), 0, false);
}

// =========================================================================
// PHẦN B — WEBSOCKET SERVER TỐI GIẢN (Winsock2 + CNG SHA1/Base64)
// Ghi chú: bản triển khai tham khảo cho LAN nội bộ, hỗ trợ frame text đơn
// (không phân mảnh), chưa hỗ trợ TLS (wss://). Đủ dùng cho dashboard giám
// sát/điều khiển AGV trong xưởng.
// =========================================================================
struct ClientConn {
    SOCKET sock = INVALID_SOCKET;
    mutex  sendMtx; // std::mutex khong the copy/move -> KHONG duoc copy struct nay,
                     // luon tao qua make_shared<ClientConn>() roi gan truong sau.
};
static vector<shared_ptr<ClientConn>> g_clients;
static mutex g_clientsMtx;

// Ma hoa Base64 bang OpenSSL (EVP_EncodeBlock) -- dung CHUNG 1 ham cho ca Windows
// va Linux/Raspberry Pi, thay the CryptBinaryToStringA (CryptoAPI, chi co tren Windows).
static string base64Encode(const unsigned char* data, int len) {
    string out((size_t)(((len + 2) / 3) * 4) + 1, '\0'); // cong 1 cho null-terminator EVP_EncodeBlock ghi kem
    int written = EVP_EncodeBlock(reinterpret_cast<unsigned char*>(&out[0]), data, len);
    out.resize((size_t)written);
    return out;
}

// SHA1 + Base64 cho bat tay WebSocket (RFC 6455) bang OpenSSL -- thay the BCrypt/CNG
// (CryptoAPI, chi co tren Windows), chay duoc tren ca Linux/Raspberry Pi.
static string sha1Base64(const string& input) {
    unsigned char hash[SHA_DIGEST_LENGTH] = {0}; // 20 byte
    SHA1(reinterpret_cast<const unsigned char*>(input.data()), input.size(), hash);
    return base64Encode(hash, SHA_DIGEST_LENGTH);
}

static bool wsHandshake(SOCKET client) {
    char buf[4096];
    int n = recv(client, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return false;
    buf[n] = 0;
    string req(buf);
    string marker = "Sec-WebSocket-Key:";
    size_t pos = req.find(marker);
    if (pos == string::npos) return false;
    pos += marker.size();
    size_t end = req.find("\r\n", pos);
    string key = req.substr(pos, end - pos);
    size_t s = key.find_first_not_of(" \t");
    size_t e = key.find_last_not_of(" \t");
    key = (s == string::npos) ? "" : key.substr(s, e - s + 1);

    string accept = sha1Base64(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11");
    string resp = "HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  "Sec-WebSocket-Accept: " + accept + "\r\n\r\n";
    return send(client, resp.c_str(), (int)resp.size(), 0) != SOCKET_ERROR;
}

static bool wsSendText(shared_ptr<ClientConn> c, const string& msg) {
    lock_guard<mutex> lk(c->sendMtx);
    vector<unsigned char> frame;
    frame.push_back(0x81); // FIN + opcode text
    size_t len = msg.size();
    if (len <= 125) frame.push_back((unsigned char)len);
    else if (len <= 65535) {
        frame.push_back(126);
        frame.push_back((len >> 8) & 0xFF); frame.push_back(len & 0xFF);
    } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; --i) frame.push_back((unsigned char)((len >> (8 * i)) & 0xFF));
    }
    frame.insert(frame.end(), msg.begin(), msg.end());
    return send(c->sock, (const char*)frame.data(), (int)frame.size(), 0) != SOCKET_ERROR;
}

// Đọc 1 frame text từ client (frame client->server luôn có mask theo chuẩn RFC6455)
static bool wsRecvText(SOCKET client, string& out, bool& closed) {
    closed = false;
    unsigned char hdr[2];
    int r = recv(client, (char*)hdr, 2, 0);
    if (r <= 0) { closed = true; return false; }
    unsigned char opcode = hdr[0] & 0x0F;
    bool masked = (hdr[1] & 0x80) != 0;
    uint64_t len = hdr[1] & 0x7F;
    if (len == 126) { unsigned char ext[2]; if (recv(client, (char*)ext, 2, 0) <= 0) { closed = true; return false; } len = (ext[0] << 8) | ext[1]; }
    else if (len == 127) { unsigned char ext[8]; if (recv(client, (char*)ext, 8, 0) <= 0) { closed = true; return false; } len = 0; for (int i = 0; i < 8; i++) len = (len << 8) | ext[i]; }

    unsigned char maskKey[4] = {0, 0, 0, 0};
    if (masked) { if (recv(client, (char*)maskKey, 4, 0) <= 0) { closed = true; return false; } }

    string payload; payload.resize((size_t)len);
    size_t got = 0;
    while (got < len) {
        int rr = recv(client, &payload[got], (int)(len - got), 0);
        if (rr <= 0) { closed = true; return false; }
        got += rr;
    }
    if (masked) for (size_t i = 0; i < len; ++i) payload[i] = (char)((unsigned char)payload[i] ^ maskKey[i % 4]);

    if (opcode == 0x8) { closed = true; return false; } // close frame
    if (opcode == 0x1) { out = payload; return true; }   // text frame
    out.clear();
    return true; // ping/pong/binary: bỏ qua trong bản demo này
}

// Xử lý 1 lệnh JSON gửi lên từ dashboard web
static void handleWebCommand(const string& msg) {
    try {
        json j = json::parse(msg);
        string cmd = j.value("cmd", "");
        if (cmd == "add_agv") {
            string id = j.at("id").get<string>();   
            string start = j.value("start", string("CHG_1"));
            g_fleet.addAGV(id, start, j.value("priority", false));
            pushLog("Da them AGV_" + id + " tai node " + start);
        } else if (cmd == "remove_agv") {
            string id = j.at("id").get<string>();
            g_fleet.removeAGV(id);
            pushLog("Da xoa AGV_" + id + " khoi he thong");
        } else if (cmd == "clear_error") {
            // Người vận hành xác nhận đã xử lý lỗi của xe -> xe về IDLE và nhận task trở lại
            string id = j.at("id").get<string>();
            if (g_fleet.clearError(id)) pushLog("Da xoa loi cho AGV_" + id + " -> xe san sang nhan task");
            else pushLog("[LOI] clear_error: AGV_" + id + " khong o trang thai LOI");
        } else if (cmd == "add_task") {
            string target = j.at("target").get<string>();
            int priority = j.value("priority", 0);
            g_fleet.addTask(target, priority);
            pushLog("Da them task toi " + target + (priority > 0 ? " (uu tien cao)" : ""));
        } else if (cmd == "cancel_task") {
            // Huy task theo dich den: huy moi task dang CHO trong hang doi + cho xe dang
            // huong toi dich do (neu co) dung lai va ve IDLE. Xem FleetManager::cancelTask.
            string target = j.at("target").get<string>();
            int n = g_fleet.cancelTask(target);
            if (n > 0) pushLog("Da huy task toi " + target + " (" + to_string(n) + " muc bi anh huong)");
            else pushLog("[LOI] cancel_task: khong tim thay task/xe nao dang huong toi " + target);
        } else if (cmd == "direct_goto") {
            // Ra lệnh trực tiếp 1 xe cụ thể (bấm chọn xe -> bấm node đích trên dashboard)
            string agvId = j.at("agv_id").get<string>();
            string target = j.at("target").get<string>();
            string err = g_fleet.directGoto(agvId, target);
            if (!err.empty()) { cout << "[DIRECT_GOTO] Loi: " << err << "\n"; pushLog("[LOI] Khong the dieu huong AGV_" + agvId + ": " + err); }
            else pushLog("Da ra lenh AGV_" + agvId + " di toi " + target);
        } else if (cmd == "add_node") {
            string id = j.at("id").get<string>();
            g_map.addNode(id, j.value("x", 0.0), j.value("y", 0.0), j.value("type", string("waypoint")));
            g_arbiter.registerZone(id);
            pushLog("Da them node " + id + " vao ban do");
        } else if (cmd == "remove_node") {
            string id = j.at("id").get<string>();
            g_map.removeNode(id);
            pushLog("Da xoa node " + id + " khoi ban do");
        } else if (cmd == "add_edge") {
            string from = j.at("from").get<string>(), to = j.at("to").get<string>();
            g_map.addEdge(from, to, j.value("weight", 1),
                          j.value("type", string("main")), j.value("path_class", string("")), j.value("two_way", false));
            pushLog("Da them duong noi " + from + " <-> " + to);
        } else if (cmd == "remove_edge") {
            string from = j.at("from").get<string>(), to = j.at("to").get<string>();
            g_map.removeEdge(from, to);
            pushLog("Da xoa duong noi " + from + " - " + to);
        } else if (cmd == "set_obstacle") {
            string node = j.at("node").get<string>();
            bool blocked = j.value("blocked", true);
            g_map.setObstacle(node, blocked);
            pushLog(blocked ? ("Da khoa node " + node + " (vat can)") : ("Da mo khoa node " + node));
        } else if (cmd == "save_map") {
            // Mac dinh luu vao thu muc data/ (xem MAP_SAVE_DIR o main()) de de tim, thay vi
            // thu muc lam viec hien tai cua tien trinh (co the la bat ky dau tuy cach chay .exe).
            string path = j.value("path", string("data/map_layout.json"));
            g_map.saveToFile(path);
            pushLog("Da luu ban do vao file: " + path);
        } else {
            cout << "[WS] Lenh khong xac dinh: " << cmd << "\n";
            pushLog("[LOI] Lenh khong xac dinh tu dashboard: " + cmd);
        }
    } catch (const std::exception& ex) {
        cout << "[WS] Loi xu ly lenh: " << ex.what() << "\n";
        pushLog(string("[LOI] Xu ly lenh that bai: ") + ex.what());
    }
}

static void wsClientThread(shared_ptr<ClientConn> c) {
    if (!wsHandshake(c->sock)) { closesocket(c->sock); return; }
    {
        lock_guard<mutex> lk(g_clientsMtx);
        g_clients.push_back(c);
    }
    // Gửi ngay snapshot hiện tại cho client mới kết nối
    json snap; snap["map"] = g_map.toJson(); 
    snap["fleet"] = g_fleet.toJson(); 
    snap["mutex"] = g_arbiter.toJson();
    snap["logs"] = recentLogsJson();
    wsSendText(c, snap.dump());

    string msg; bool closed = false;
    while (wsRecvText(c->sock, msg, closed)) {
        if (!msg.empty()) handleWebCommand(msg);
    }
    closesocket(c->sock);
    lock_guard<mutex> lk(g_clientsMtx);
    g_clients.erase(remove_if(g_clients.begin(), g_clients.end(),
                    [&](const shared_ptr<ClientConn>& x) { 
                        return x.get() == c.get(); }), g_clients.end());
}

static void wsAcceptLoop(SOCKET listenSock) {
    while (true) {
        SOCKET client = accept(listenSock, nullptr, nullptr);
        if (client == INVALID_SOCKET) continue;
        auto conn = make_shared<ClientConn>(); // tao rong, KHONG copy (mutex ben trong khong copy duoc)
        conn->sock = client;
        thread(wsClientThread, conn).detach();
    }
}

static void broadcastState() {
    json snap; snap["map"] = g_map.toJson(); snap["fleet"] = g_fleet.toJson(); snap["mutex"] = g_arbiter.toJson();
    snap["logs"] = recentLogsJson(); // nhat ky su kien gan nhat, xem "NHAT KY SU KIEN HE THONG" o Phan A
    string payload = snap.dump();
    vector<shared_ptr<ClientConn>> clientsCopy;
    { lock_guard<mutex> lk(g_clientsMtx); clientsCopy = g_clients; }
    for (auto& c : clientsCopy) wsSendText(c, payload);
}

static SOCKET startWebSocketServer(int port) {
#if defined(_WIN32)
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { cerr << "[WS] WSAStartup that bai\n"; return INVALID_SOCKET; }
#endif

    SOCKET listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock == INVALID_SOCKET) {
        cerr << "[WS] Khong the tao socket lang nghe\n";
        return INVALID_SOCKET;
    }

    // Cho phép Core khởi động lại ngay sau khi tắt mà không bị kẹt cổng 8080
    // bởi các kết nối WebSocket vừa đóng. int 1/0 tuong thich ca Winsock (BOOL
    // that chat la typedef cua int) lan POSIX (setsockopt nhan int cho co nay).
    int reuseAddress = 1;
    setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuseAddress), sizeof(reuseAddress));

    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_addr.s_addr = INADDR_ANY; addr.sin_port = htons((uint16_t)port);
    if (bind(listenSock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        cerr << "[WS] Bind that bai tren port " << port << "\n";
        closesocket(listenSock);
        return INVALID_SOCKET;
    }
    if (listen(listenSock, SOMAXCONN) == SOCKET_ERROR) {
        cerr << "[WS] Listen that bai tren port " << port << "\n";
        closesocket(listenSock);
        return INVALID_SOCKET;
    }
    cout << "[WS] C++ Core WebSocket dang lang nghe tren ws://0.0.0.0:" << port << "\n";
    thread(wsAcceptLoop, listenSock).detach();
    return listenSock;
}

// =========================================================================
// PHẦN C — MQTT (giao tiếp với AGV qua ESP32)
// =========================================================================
static bool parseTopic(const string& topic, string& agvId, string& action) {
    stringstream ss(topic); string part; vector<string> parts;
    while (getline(ss, part, '/')) parts.push_back(part);
    if (parts.size() != 3 || parts[0] != "agv") return false;
    agvId = parts[1]; action = parts[2]; return true;
}

static void on_connect(mosquitto* mosq, void*, int rc) {
    static bool hasConnected= false;
    if (rc == 0) {
        if(!hasConnected){
        cout << "[MQTT] Ket noi Broker thanh cong! Dang lang nghe cac xe AGV...\n";
        hasConnected= true;
        }
        mosquitto_subscribe(mosq, NULL, "agv/+/telemetry", 0);
        mosquitto_subscribe(mosq, NULL, "agv/+/mutex_request", 0);
        mosquitto_subscribe(mosq, NULL, "agv/+/mutex_release", 0);
        mosquitto_subscribe(mosq, NULL, "agv/+/obstacle", 0);
        mosquitto_subscribe(mosq, NULL, "agv/+/error", 0);
        mosquitto_subscribe(mosq, NULL, "agv/+/charging_done", 0);
        mosquitto_subscribe(mosq, NULL, "agv/+/ack", 0);
    } else {
        cerr << "[MQTT] Ket noi that bai, ma loi: " << rc << "\n";
        hasConnected= false;
    }
}

static void on_message(mosquitto*, void*, const mosquitto_message* msg) {
    string topic = msg->topic;
    string payload(static_cast<char*>(msg->payload), msg->payloadlen);
    string agvId, action;
    if (!parseTopic(topic, agvId, action)) return;

    if (action == "telemetry") {
        // Cu phap: "RFID|battery" hoac "RFID|battery|obstacle(0/1)"
        vector<string> parts; stringstream ss(payload); string tok;
        while (getline(ss, tok, '|')) parts.push_back(tok);
        if (parts.size() >= 2) {
            string rfid = parts[0];
            int battery = 0; try { battery = stoi(parts[1]); } catch (...) {}
            bool obstacle = (parts.size() >= 3 && parts[2] == "1");
            g_fleet.updateTelemetry(agvId, rfid, battery, obstacle);
            if (obstacle) requestWake(); // vat can bao qua telemetry -> replanning ngay, khong doi het 700ms
        }
    } else if (action == "mutex_request") {
        if (g_arbiter.requestZone(payload, agvId)) publishTo("agv/" + agvId + "/mutex_grant", payload);
    } else if (action == "mutex_release") {
        string nextAgv = g_arbiter.releaseZone(payload, agvId);
        if (!nextAgv.empty()) publishTo("agv/" + nextAgv + "/mutex_grant", payload);
    } else if (action == "obstacle") {
        g_fleet.reportObstacle(agvId, payload);
        cout << "[OBSTACLE] AGV_" << agvId << " phat hien vat can tai " << payload << "\n";
        pushLog("AGV_" + agvId + " phat hien vat can tai node " + payload);
        requestWake(); // danh thuc vong lap chinh ngay -- xem "CO CHE HUONG SU KIEN" o dau file
    } else if (action == "error") {
        g_fleet.reportError(agvId, payload);
        cout << "[ALARM] AGV_" << agvId << " BAO LOI: " << payload << "\n";
        requestWake(); // loi xe cung can xu ly (huy task, tra hang doi...) va broadcast ngay lap tuc
    } else if (action == "charging_done") {
        // payload rong -> dung target mac dinh; neu ESP32 gui kem % pin that thi dung so do
        int finalBattery = -1;
        if (!payload.empty()) { try { finalBattery = stoi(payload); } catch (...) {} }
        g_fleet.reportCharged(agvId, finalBattery);
        cout << "[CHARGE] AGV_" << agvId << " da sac xong.\n";
        pushLog("AGV_" + agvId + " da sac xong (" + (finalBattery >= 0 ? to_string(finalBattery) : string("~90")) + "% pin)");
    } else if (action == "ack") {
        try { g_fleet.reportAck(agvId, stol(payload)); } catch (...) {}
    }
}

static bool startMqtt(const string& brokerIp, int port) {
    mosquitto_lib_init();
    g_mosq = mosquitto_new("AGV_MasterServer", true, NULL);
    if (!g_mosq) { cerr << "[MQTT] Khong the khoi tao mosquitto client!\n"; return false; }
    mosquitto_connect_callback_set(g_mosq, on_connect);
    mosquitto_message_callback_set(g_mosq, on_message);
    if (mosquitto_connect(g_mosq, brokerIp.c_str(), port, 60) != MOSQ_ERR_SUCCESS) {
        cerr << "[MQTT] Khong the ket noi den Broker tai " << brokerIp << ":" << port << "\n";
        return false;
    }
    mosquitto_loop_start(g_mosq);
    return true;
}

// =========================================================================
// PHẦN D — MAIN
// =========================================================================
int main() {
    // Bat Ctrl+C (SIGINT) / lenh kill (SIGTERM) de thoat vong lap chinh sach se tren
    // MOI nen tang, thay the GetAsyncKeyState(VK_ESCAPE) truoc day chi dung tren Windows.
    std::signal(SIGINT, onShutdownSignal);
    std::signal(SIGTERM, onShutdownSignal);

    // 0) Bao dam thu muc data/ ton tai -- day la noi CO DINH server luu/nap ban do,
    //    thay vi thu muc lam viec hien tai cua tien trinh (thay doi tuy cach chay .exe,
    //    dan den nguoi dung khong biet "luu vao dau"). Duong dan: data/map_layout.json.
    static const string MAP_SAVE_DIR  = "data";
    static const string MAP_SAVE_PATH = MAP_SAVE_DIR + "/map_layout.json";
    std::filesystem::create_directories(MAP_SAVE_DIR); // tao thu muc, bo qua neu da ton tai san (cross-platform)

    // 1) Nap ban do: uu tien data/map_layout.json (duong dan chuan tu ban nay tro di).
    //    Neu chua co (VD: file cu luu o thu muc goc tu ban truoc), thu nap file cu do
    //    roi LUU LAI ngay vao data/ de tu dong "don" ve 1 noi duy nhat. Neu khong co gi
    //    ca thi dung map demo khop so do nha xuong da thiet ke.
    try {
        g_map.loadFromFile(MAP_SAVE_PATH);
        cout << "[MAP] Da nap " << MAP_SAVE_PATH << "\n";
    } catch (const std::exception&) {
        try {
            g_map.loadFromFile("map_layout.json"); // duong dan cu (tuong thich nguoc)
            cout << "[MAP] Da nap map_layout.json (duong dan cu) -> se luu lai vao " << MAP_SAVE_PATH << "\n";
            g_map.saveToFile(MAP_SAVE_PATH);
        } catch (const std::exception&) {
            cout << "[MAP] Khong tim thay ban do da luu -> dung map demo mac dinh.\n";
            g_map.loadDefaultDemoMap();
        }
    }
    for (auto& z : g_map.mutexZones()) g_arbiter.registerZone(z);

    // 2) Nối FleetManager với kênh phát lệnh MQTT + kênh báo thay đổi -> WebSocket
    g_fleet.setPublishCallback(publishTo);
    g_fleet.setStateChangedCallback(broadcastState);
    g_fleet.setChargeThresholds(20, 40, 90); // <=20% -> di sac; can >=40% moi nhan task moi; sac xong ~90% neu ESP32 khong bao pin that
    g_fleet.setOfflineTimeout(10);    // >10s khong co telemetry -> xe OFFLINE, task duoc tra ve hang doi
    g_fleet.setAckTimeout(3000);             // 3s khong thay ACK lenh -> gui lai
    g_fleet.setMaxRetries(3);                // gui lai toi da 3 lan truoc khi coi la mat lien lac lenh
    g_fleet.setEventCallback([](const string& msg) { cout << "[FLEET] " << msg << "\n"; pushLog(msg); });

    // 3) Khởi động MQTT (giao tiếp AGV) — chỉnh IP broker tại đây nếu cần
    if (!startMqtt("127.0.0.1", 1883)) {
        cerr << "[LOI] Khong khoi dong duoc MQTT, thoat chuong trinh.\n";
        return 1;
    }

    // 4) Khởi động WebSocket cho C++ Core (Node.js gateway sẽ kết nối vào đây)
    if (startWebSocketServer(WS_PORT) == INVALID_SOCKET) {
        cerr << "[LOI] Khong khoi dong duoc WebSocket server.\n";
        return 1;
    }

    cout << "\n=== AGV MASTER SERVER DANG CHAY ===\n";
    cout << "MQTT broker : 127.0.0.1:1883\n";
    cout << "C++ Core   : ws://<IP-may-nay>:" << WS_PORT << "\n";
    cout << "Ban do luu tai: " << MAP_SAVE_PATH << " (thu muc lam viec hien tai: xem the tren)\n";
    cout << "Nhan Ctrl+C de thoat.\n\n";

    // 5) Vòng lặp chính: tick() định kỳ (dispatch + kiểm tra pin), thoát bằng Ctrl+C (g_running).
    //    Dùng condition_variable thay vì sleep_for cố định: chờ tối đa 700ms NHƯNG được đánh
    //    thức SỚM HƠN ngay khi luồng MQTT gọi requestWake() (vật cản/lỗi từ ESP32) -- loại bỏ
    //    độ trễ tĩnh khi cần replanning gấp, mà vẫn giữ 700ms làm chu kỳ tick() nền bình thường.
    while (g_running) {
        g_fleet.tick();
        broadcastState();

        std::unique_lock<std::mutex> wakeLock(g_wakeMtx);
        g_wakeCv.wait_for(wakeLock, chrono::milliseconds(700),
                           [] { return g_wakeRequested.load() || !g_running.load(); });
        g_wakeRequested = false;
    }
    cout << "[CONTROL] Dang thoat chuong trinh...\n";

    // 6) Dọn dẹp
    mosquitto_loop_stop(g_mosq, true);
    mosquitto_disconnect(g_mosq);
    mosquitto_destroy(g_mosq);
    mosquitto_lib_cleanup();
#if defined(_WIN32)
    WSACleanup();
#endif
    return 0;
}