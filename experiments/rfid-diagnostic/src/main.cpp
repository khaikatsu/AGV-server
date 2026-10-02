#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include "../../../NCKHAGV/src/Config.h"

namespace {
MFRC522 reader(Config::RFID_SS_PIN, Config::RFID_RST_PIN);
uint32_t lastProbeMs = 0;
uint32_t probeCount = 0;

void printRegisters() {
    Serial.print("VersionReg: ");
    for (int i = 0; i < 8; ++i) {
        const byte value = reader.PCD_ReadRegister(MFRC522::VersionReg);
        Serial.printf("%02X%s", value, i == 7 ? "" : " ");
        delay(5);
    }
    const byte tx = reader.PCD_ReadRegister(MFRC522::TxControlReg);
    Serial.printf(" | TxControlReg=%02X | antenna=%s\n",
                  tx, (tx & 0x03) == 0x03 ? "ON" : "OFF");
}

void printUid() {
    Serial.print("UID=");
    for (byte i = 0; i < reader.uid.size; ++i)
        Serial.printf("%02X", reader.uid.uidByte[i]);
    Serial.println();
}
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("=== RC522 DIAGNOSTIC ===");
    Serial.printf("ESP32: SCK=%d MISO=%d MOSI=%d SS=%d RST=%d\n",
                  Config::RFID_SCK_PIN, Config::RFID_MISO_PIN,
                  Config::RFID_MOSI_PIN, Config::RFID_SS_PIN, Config::RFID_RST_PIN);
    SPI.begin(Config::RFID_SCK_PIN, Config::RFID_MISO_PIN,
              Config::RFID_MOSI_PIN, Config::RFID_SS_PIN);
    reader.PCD_Init();
    delay(50);
    reader.PCD_AntennaOn();
    reader.PCD_SetAntennaGain(MFRC522::RxGain_max);
    printRegisters();
    Serial.println("Dat DUNG MOT the 13.56 MHz sat mat anten. Moi giay se co ket qua.");
}

void loop() {
    const uint32_t now = millis();
    if (uint32_t(now - lastProbeMs) < 1000) return;
    lastProbeMs = now;
    ++probeCount;

    if (probeCount % 5 == 0) printRegisters();

    const byte txBefore = reader.PCD_ReadRegister(MFRC522::TxControlReg);
    if ((txBefore & 0x03) != 0x03) {
        const byte version = reader.PCD_ReadRegister(MFRC522::VersionReg);
        Serial.printf("CANH BAO: antenna tu OFF, Tx=%02X Version=%02X RST_GPIO=%d\n",
                      txBefore, version, digitalRead(Config::RFID_RST_PIN));
        reader.PCD_AntennaOn();
        const byte txAfter = reader.PCD_ReadRegister(MFRC522::TxControlReg);
        Serial.printf("Bat lai antenna: Tx=%02X (%s)\n", txAfter,
                      (txAfter & 0x03) == 0x03 ? "ON" : "VAN OFF");
    }

    byte atqa[2] = {};
    byte atqaSize = sizeof(atqa);
    // WUPA can see a card that was left against the antenna after a prior HALT.
    const MFRC522::StatusCode status = reader.PICC_WakeupA(atqa, &atqaSize);
    Serial.printf("Probe %lu: WUPA=%u ",
                  static_cast<unsigned long>(probeCount), unsigned(status));
    Serial.println(MFRC522::GetStatusCodeName(status));

    if (status == MFRC522::STATUS_TIMEOUT) {
        Serial.println("Khong co phan hoi RF tu the.");
        return;
    }
    if (status != MFRC522::STATUS_OK) return;

    Serial.printf("ATQA=%02X%02X\n", atqa[0], atqa[1]);
    if (!reader.PICC_ReadCardSerial()) {
        Serial.println("Co phan hoi RF nhung khong doc duoc UID.");
        return;
    }
    printUid();
    reader.PICC_HaltA();
    reader.PCD_StopCrypto1();
}
