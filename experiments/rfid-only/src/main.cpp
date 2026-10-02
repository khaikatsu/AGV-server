#include <Arduino.h>
#include <MFRC522.h>
#include <Preferences.h>
#include <SPI.h>

#include "../../../NCKHAGV/src/Config.h"

namespace {

constexpr char NVS_NAMESPACE[] = "rfid-test";
constexpr char NVS_CARD_1_KEY[] = "card1";

MFRC522 reader(Config::RFID_SS_PIN, Config::RFID_RST_PIN);
Preferences preferences;
String card1Uid;
bool setCard1Mode = false;

void printHelp() {
  Serial.println();
  Serial.println("===== RFID TEST =====");
  Serial.println("s: doc the, chi in UID");
  Serial.println("1: gan the vua quet thanh THE 1 va luu sau reset");
  Serial.println("c: xoa quyen THE 1");
  Serial.println("p: in UID THE 1 hien tai");
  Serial.println("?: hien huong dan");
  Serial.println("=====================");
}

void printCard1() {
  if (card1Uid.isEmpty()) {
    Serial.println("THE 1: CHUA SET");
  } else {
    Serial.printf("THE 1 UID=%s\n", card1Uid.c_str());
  }
}

String readUid() {
  String uid;
  uid.reserve(reader.uid.size * 2);
  const char hex[] = "0123456789ABCDEF";
  for (byte i = 0; i < reader.uid.size; ++i) {
    const byte value = reader.uid.uidByte[i];
    uid += hex[value >> 4];
    uid += hex[value & 0x0F];
  }
  return uid;
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(tolower(Serial.read()));
    switch (command) {
      case '1':
        setCard1Mode = true;
        Serial.println("SET MODE: dua the muon cap quyen THE 1 vao RC522");
        break;

      case 's':
        setCard1Mode = false;
        Serial.println("SCAN MODE: dua the vao RC522");
        break;

      case 'c':
        setCard1Mode = false;
        card1Uid = "";
        if (preferences.isKey(NVS_CARD_1_KEY)) {
          preferences.remove(NVS_CARD_1_KEY);
        }
        Serial.println("DA XOA QUYEN THE 1");
        break;

      case 'p':
        printCard1();
        break;

      case '?':
        printHelp();
        break;

      default:
        break;
    }
  }
}

void pollCard() {
  if (!reader.PICC_IsNewCardPresent() || !reader.PICC_ReadCardSerial()) {
    return;
  }

  const String uid = readUid();
  Serial.printf("UID=%s\n", uid.c_str());

  if (setCard1Mode) {
    card1Uid = uid;
    preferences.putString(NVS_CARD_1_KEY, card1Uid);
    setCard1Mode = false;
    Serial.printf("DA SET THE 1=%s\n", card1Uid.c_str());
  } else if (!card1Uid.isEmpty() && uid == card1Uid) {
    Serial.println("THE 1 HOP LE");
  } else {
    Serial.println("THE KHONG DUOC CAP QUYEN");
  }

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();
}

}  // namespace

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  delay(300);

  SPI.begin(Config::RFID_SCK_PIN, Config::RFID_MISO_PIN,
            Config::RFID_MOSI_PIN, Config::RFID_SS_PIN);
  reader.PCD_Init();
  delay(50);
  reader.PCD_AntennaOn();
  reader.PCD_SetAntennaGain(MFRC522::RxGain_max);

  preferences.begin(NVS_NAMESPACE, false);
  if (preferences.isKey(NVS_CARD_1_KEY)) {
    card1Uid = preferences.getString(NVS_CARD_1_KEY, "");
  }

  const byte version = reader.PCD_ReadRegister(MFRC522::VersionReg);
  const byte txControl = reader.PCD_ReadRegister(MFRC522::TxControlReg);
  Serial.printf("RC522 SCK=%d MISO=%d MOSI=%d SDA/SS=%d RST=%d\n",
                Config::RFID_SCK_PIN, Config::RFID_MISO_PIN,
                Config::RFID_MOSI_PIN, Config::RFID_SS_PIN,
                Config::RFID_RST_PIN);
  Serial.printf("VersionReg=0x%02X | antenna=%s\n", version,
                (txControl & 0x03) == 0x03 ? "ON" : "OFF");
  printHelp();
  printCard1();
  Serial.println("SCAN MODE: dua the vao RC522");
}

void loop() {
  handleSerial();
  pollCard();
  delay(2);
}
