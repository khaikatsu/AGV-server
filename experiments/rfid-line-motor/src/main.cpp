#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <cstring>
#include "TestConfig.h"
#include "TestLogic.h"

namespace {
enum class Driver { Unconfigured, L298N, BTS7960 };
Driver driver = Driver::Unconfigured;
int activeLine = -1;
MFRC522 reader(Config::RFID_SS_PIN, Config::RFID_RST_PIN);
Bench::Controller controller({Bench::BASE_PWM, Bench::CORRECTION_PWM,
    Bench::TURN_PWM, Bench::TURN_MIN_MS, Bench::TURN_TIMEOUT_MS,
    Bench::LINE_STABLE_MS, Bench::CARD_1_PIVOT_RIGHT});
char card1Uid[21] = {};
bool learnCard = false;
bool manual = false;
uint32_t manualStarted = 0;
uint32_t lastPoll = 0;
uint32_t lastReport = 0;
Bench::Output applied;

bool validUid(const char* uid) {
    const size_t n = strlen(uid);
    if (n != 8 && n != 14 && n != 20) return false;
    for (size_t i = 0; i < n; ++i)
        if (!((uid[i] >= '0' && uid[i] <= '9') || (uid[i] >= 'A' && uid[i] <= 'F')))
            return false;
    return true;
}
void writeSide(bool left, int duty) {
    const int enable = left ? Config::LEFT_ENA : Config::RIGHT_ENB;
    const int a = left ? Config::LEFT_IN1 : Config::RIGHT_IN3;
    const int b = left ? Config::LEFT_IN2 : Config::RIGHT_IN4;
    if (left ? Bench::LEFT_FORWARD_INVERTED : Bench::RIGHT_FORWARD_INVERTED)
        duty = -duty;
    duty = constrain(duty, -255, 255);
    if (driver == Driver::L298N) {
        const int channel = left ? 0 : 1;
        ledcWrite(channel, 0);
        digitalWrite(a, duty > 0);
        digitalWrite(b, duty < 0);
        ledcWrite(channel, abs(duty));
    } else if (driver == Driver::BTS7960) {
        const int channel = left ? 0 : 2;
        digitalWrite(enable, LOW);
        ledcWrite(channel, 0);
        ledcWrite(channel + 1, 0);
        if (duty) {
            ledcWrite(channel + (duty < 0), abs(duty));
            digitalWrite(enable, HIGH);
        }
    } else {
        digitalWrite(enable, LOW);
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
    }
}
void drive(Bench::Output output) {
    writeSide(true, output.left);
    writeSide(false, output.right);
    applied = output;
}
void stop() {
    manual = false;
    controller.stop();
    drive({});
}
void selectDriver(Driver selected) {
    stop();
    // Disable/detach previous PWM before changing the bench adapter.
    if (driver == Driver::L298N) {
        ledcDetachPin(Config::LEFT_ENA); ledcDetachPin(Config::RIGHT_ENB);
    } else if (driver == Driver::BTS7960) {
        for (int pin : {Config::LEFT_IN1, Config::LEFT_IN2,
                        Config::RIGHT_IN3, Config::RIGHT_IN4}) ledcDetachPin(pin);
    }
    for (int pin : {Config::LEFT_ENA, Config::LEFT_IN1, Config::LEFT_IN2,
                    Config::RIGHT_ENB, Config::RIGHT_IN3, Config::RIGHT_IN4}) {
        digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
    }
    driver = selected;
    for (int channel = 0; channel < 4; ++channel) {
        ledcSetup(channel, 5000, 8); ledcWrite(channel, 0);
    }
    if (driver == Driver::L298N) {
        ledcAttachPin(Config::LEFT_ENA, 0); ledcAttachPin(Config::RIGHT_ENB, 1);
    } else if (driver == Driver::BTS7960) {
        ledcAttachPin(Config::LEFT_IN1, 0); ledcAttachPin(Config::LEFT_IN2, 1);
        ledcAttachPin(Config::RIGHT_IN3, 2); ledcAttachPin(Config::RIGHT_IN4, 3);
    }
    drive({});
    Serial.println(driver == Driver::L298N ? "Driver=L298N, dung motor" :
                   driver == Driver::BTS7960 ? "Driver=BTS7960, dung motor" : "Driver chua chon");
}
Bench::Line readLine() {
    return {activeLine >= 0 && digitalRead(Config::LINE_LEFT_PIN) == activeLine,
            activeLine >= 0 && digitalRead(Config::LINE_CENTER_PIN) == activeLine,
            activeLine >= 0 && digitalRead(Config::LINE_RIGHT_PIN) == activeLine};
}
void help() {
    Serial.println("a=L298N; b=BTS7960; 0=line LOW; 1=line HIGH");
    Serial.println("u=dung va hoc UID the 1 (quet tiep); g=chay; s=DUNG");
    Serial.println("f=tien 400ms; r=quay phai 400ms; l=quay trai 400ms; ?=help");
    Serial.println("UID hoc chi luu RAM. Moi lan g chi re theo the 1 mot lan.");
}
void serialCommands(uint32_t now) {
    for (int count = 0; count < 32 && Serial.available(); ++count) {
        const char command = Serial.read();
        switch (command) {
            case 's':
                stop(); learnCard = false;
                while (Serial.available()) Serial.read();
                Serial.println("DUNG"); return;
            case 'a': learnCard = false; selectDriver(Driver::L298N); break;
            case 'b': learnCard = false; selectDriver(Driver::BTS7960); break;
            case '0': case '1':
                stop(); activeLine = command - '0';
                Serial.printf("Line active=%d\n", activeLine); break;
            case 'u':
                stop(); learnCard = true;
                Serial.println("Quet the 1 de hoc UID (neu vua quet, nhac the ra roi dua lai)"); break;
            case 'g':
                if (manual || controller.state() == Bench::State::Turning ||
                    controller.state() == Bench::State::Follow) {
                    Serial.println("Dung bang s truoc khi bat dau lai"); break;
                }
                if (driver == Driver::Unconfigured || activeLine < 0 || !validUid(card1Uid)) {
                    Serial.println("Chua san sang: chon a/b, chon 0/1, hoc UID bang u"); break;
                }
                learnCard = false; controller.start();
                Serial.println("CHAY DO LINE"); break;
            case 'f': case 'r': case 'l':
                stop(); learnCard = false;
                if (driver == Driver::Unconfigured) {
                    Serial.println("Chon driver a/b truoc"); break;
                }
                manual = true; manualStarted = now;
                drive(command == 'f' ? Bench::Output{Bench::BASE_PWM, Bench::BASE_PWM} :
                      command == 'r' ? Bench::Output{Bench::TURN_PWM, -Bench::TURN_PWM} :
                                       Bench::Output{-Bench::TURN_PWM, Bench::TURN_PWM}); break;
            case '?': help(); break;
            default: break;
        }
    }
}
void pollCard(uint32_t now, Bench::Line line) {
    if (manual || controller.state() == Bench::State::Turning ||
        uint32_t(now - lastPoll) < Bench::RFID_POLL_MS) return;
    lastPoll = now;
    if (!reader.PICC_IsNewCardPresent() || !reader.PICC_ReadCardSerial()) return;
    char uid[21] = {};
    const size_t n = reader.uid.size;
    if (n == 4 || n == 7 || n == 10) {
        const char* hex = "0123456789ABCDEF";
        for (size_t i = 0; i < n; ++i) {
            uid[2 * i] = hex[reader.uid.uidByte[i] >> 4];
            uid[2 * i + 1] = hex[reader.uid.uidByte[i] & 15];
        }
    }
    reader.PICC_HaltA(); reader.PCD_StopCrypto1();
    if (!validUid(uid)) return;
    Serial.printf("UID=%s\n", uid);
    if (learnCard) {
        strcpy(card1Uid, uid); learnCard = false;
        Serial.printf("Da gan THE 1=%s. Bam g de chay.\n", card1Uid);
    } else if (strcmp(uid, card1Uid) == 0 && controller.card1(now, line)) {
        // Start the pivot on this loop; followLine cannot overwrite it.
        Serial.println("THE 1: quay tai cho, cho LEFT roi line va bat lai line moi");
    } else Serial.println("The khong co hanh dong / the 1 da dung trong luot nay");
}
}

void setup() {
    // Drive outputs LOW before RFID or Serial initialization.
    for (int pin : {Config::LEFT_ENA, Config::LEFT_IN1, Config::LEFT_IN2,
                    Config::RIGHT_ENB, Config::RIGHT_IN3, Config::RIGHT_IN4}) {
        digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
    }
    pinMode(Config::LINE_LEFT_PIN, INPUT);
    pinMode(Config::LINE_CENTER_PIN, INPUT);
    pinMode(Config::LINE_RIGHT_PIN, INPUT);
    Serial.begin(115200);
    SPI.begin(Config::RFID_SCK_PIN, Config::RFID_MISO_PIN,
              Config::RFID_MOSI_PIN, Config::RFID_SS_PIN);
    reader.PCD_Init();
    if (validUid(Bench::CARD_1_UID)) strcpy(card1Uid, Bench::CARD_1_UID);
    help();
}
void loop() {
    uint32_t now = millis();
    serialCommands(now);
    if (manual) {
        if (uint32_t(now - manualStarted) >= Bench::MANUAL_PULSE_MS) stop();
    } else {
        auto line = readLine();
        const auto before = controller.state();
        // Check lost/ambiguous line BEFORE potentially blocking SPI polling.
        drive(controller.update(now, line));
        pollCard(now, line);
        now = millis();
        line = readLine();
        drive(controller.update(now, line));
        if (before != controller.state())
            Serial.printf("State=%u fault=%u (1=mat line, 2=line 101, 3=timeout quay)\n",
                unsigned(controller.state()), unsigned(controller.fault()));
    }
    if (uint32_t(now - lastReport) >= 250) {
        lastReport = now;
        const auto line = readLine();
        Serial.printf("raw LCR=%d%d%d line=%d%d%d PWM=%d,%d state=%u\n",
            digitalRead(Config::LINE_LEFT_PIN), digitalRead(Config::LINE_CENTER_PIN),
            digitalRead(Config::LINE_RIGHT_PIN), line.left, line.center, line.right,
            applied.left, applied.right, unsigned(controller.state()));
    }
    delay(1);
}
