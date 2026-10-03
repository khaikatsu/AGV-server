#include "RFIDModule.h"
#include "core/ControlLogic.h"
#include <SPI.h>
#include <cstring>

bool RFIDModule::validUid(const char* uid) {
    const size_t length = std::strlen(uid);
    if (length != 8 && length != 14 && length != 20) return false;
    for (size_t i = 0; i < length; ++i)
        if (!((uid[i] >= '0' && uid[i] <= '9') || (uid[i] >= 'A' && uid[i] <= 'F'))) return false;
    return true;
}
void RFIDModule::begin() {
    SPI.begin(Config::RFID_SCK_PIN, Config::RFID_MISO_PIN, Config::RFID_MOSI_PIN, Config::RFID_SS_PIN);
    reader_.PCD_Init();
    for (size_t i = 0; i < Config::RFID_NODES.size(); ++i) {
        const char* uid = Config::RFID_NODES[i].uid;
        if (!uid[0]) continue;
        if (!validUid(uid)) mappingsValid_ = false;
        for (size_t j = i + 1; j < Config::RFID_NODES.size(); ++j)
            if (std::strcmp(uid, Config::RFID_NODES[j].uid) == 0) mappingsValid_ = false;
    }
    if (!mappingsValid_ && Config::DEBUG_ENABLED) Serial.println("[RFID] Invalid/duplicate UID configuration");
}
const char* RFIDModule::getNodeFromUID(const char* uid) const {
    if (!mappingsValid_ || !uid || !validUid(uid)) return "";
    for (const auto& mapping : Config::RFID_NODES)
        if (std::strcmp(uid, mapping.uid) == 0) return mapping.node;
    return "";
}
bool RFIDModule::hasMapping(const char* node) const {
    if (!mappingsValid_) return false;
    for (const auto& mapping : Config::RFID_NODES)
        if (std::strcmp(node, mapping.node) == 0) return validUid(mapping.uid);
    return false;
}
bool RFIDModule::readCard() {
    event_.newNode = false;
    if (static_cast<uint32_t>(millis() - lastPollMs_) < Config::RFID_POLL_INTERVAL_MS) return false;
    lastPollMs_ = millis();
    if (!reader_.PICC_IsNewCardPresent() || !reader_.PICC_ReadCardSerial()) return false;
    if (reader_.uid.size != 4 && reader_.uid.size != 7 && reader_.uid.size != 10) {
        reader_.PICC_HaltA();
        reader_.PCD_StopCrypto1();
        return false;
    }
    const char* hex = "0123456789ABCDEF";
    for (size_t i = 0; i < reader_.uid.size; ++i) {
        event_.uid[i * 2] = hex[reader_.uid.uidByte[i] >> 4];
        event_.uid[i * 2 + 1] = hex[reader_.uid.uidByte[i] & 15];
    }
    event_.uid[reader_.uid.size * 2] = '\0';
    AGVMath::copyText(event_.node, sizeof(event_.node), getNodeFromUID(event_.uid));
    event_.newNode = event_.node[0] && std::strcmp(event_.node, lastNode_) != 0;
    if (!event_.node[0]) lastNode_[0] = '\0';
    if (event_.newNode) AGVMath::copyText(lastNode_, sizeof(lastNode_), event_.node);
    reader_.PICC_HaltA();
    reader_.PCD_StopCrypto1();
    if (Config::DEBUG_ENABLED) Serial.printf("[RFID] UID=%s node=%s new=%d\n", event_.uid, event_.node, event_.newNode);
    return true;
}
