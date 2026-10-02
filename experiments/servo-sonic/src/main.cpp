#include <Arduino.h>
#include "../../../NCKHAGV/src/Config.h"

namespace {
constexpr int kServoChannel = Config::SERVO_PWM_CHANNELS[0]; // channel 4, 50 Hz
constexpr int kMinAngle = 0;
constexpr int kMaxAngle = 180;
constexpr int kStepDegrees = 15;
constexpr int kHomeButtonPin = 0; // Built-in BOOT button, test project only
constexpr uint32_t kButtonDebounceMs = 30;
constexpr uint32_t kCenterWaitMs = 800;
constexpr uint32_t kSettleMs = 350;
constexpr uint32_t kEchoTimeoutUs = 25000;
constexpr uint32_t kServoMinUs = 1000; // conservative bench values, not MG90S calibration
constexpr uint32_t kServoMaxUs = 2000;

enum class ScanState { Idle, Centering, Settling, Parking };
ScanState state = ScanState::Idle;
int selectedServo = 0; // Servo 1 / GPIO15 by default; no pulse before a command
int attachedServo = -1;
int angle = 90;
int direction = 1;
uint32_t movedAtMs = 0;
bool attached = false;
int lastButtonRaw = HIGH;
int buttonStable = HIGH;
uint32_t buttonChangedAtMs = 0;

int selectedPin() { return Config::SERVO_PINS[static_cast<size_t>(selectedServo)]; }
int attachedPin() { return Config::SERVO_PINS[static_cast<size_t>(attachedServo)]; }

void attachServo() {
    if (attached) return;
    ledcSetup(kServoChannel, Config::SERVO_PWM_HZ, Config::SERVO_PWM_BITS);
    ledcWrite(kServoChannel, 0);
    ledcAttachPin(selectedPin(), kServoChannel);
    attachedServo = selectedServo;
    attached = true;
}

void setAngle(int degrees) {
    const uint32_t pulseUs = kServoMinUs +
        static_cast<uint32_t>(degrees) * (kServoMaxUs - kServoMinUs) / 180;
    const uint32_t duty =
        static_cast<uint64_t>(pulseUs) * Config::SERVO_PWM_HZ *
        ((1UL << Config::SERVO_PWM_BITS) - 1) / 1000000UL;
    ledcWrite(kServoChannel, duty);
    angle = degrees;
    movedAtMs = millis();
}

void stopServo() {
    if (!attached) {
        state = ScanState::Idle;
        Serial.println("SERVO DA TAT");
        return;
    }
    setAngle(90);
    state = ScanState::Parking;
    Serial.println("DUNG QUET: dang ve 90 do, sau do se tat xung servo");
}

void returnToZero() {
    state = ScanState::Idle;
    attachServo();
    setAngle(0);
    Serial.println("VE 0 DO: dung quet, servo giu tai goc lenh 0 do");
}

void help() {
    Serial.println("1/2/3: chon Servo 1/2/3 (GPIO15/17/21)");
    Serial.println("g: dua ve giua roi quet 0..180 do lien tuc, buoc 15 do");
    Serial.println("c: dua ve giua 90 do va giu tai do");
    Serial.println("0 hoac nut BOOT: dung quet, dua servo ve 0 do va giu");
    Serial.println("s: dung quet, ve 90 do, cho 800 ms roi tat xung; ?: huong dan");
    Serial.println("Mac dinh Servo 1/GPIO15. Khoi dong KHONG lam servo chuyen dong.");
}

void processSerial() {
    while (Serial.available()) {
        const char command = Serial.read();
        if (command == 's') {
            stopServo();
            while (Serial.available()) Serial.read(); // s has priority in this input batch
            return;
        }
        if (command == '0') {
            returnToZero();
            while (Serial.available()) Serial.read(); // home has priority in this input batch
            return;
        }
        if (command >= '1' && command <= '3') {
            if (attached) {
                Serial.println("Servo dang hoat dong; gui s va doi servo tat truoc khi doi chan");
                continue;
            }
            selectedServo = command - '1';
            Serial.printf("Da chon Servo %d, signal GPIO%d\n",
                          selectedServo + 1, selectedPin());
        } else if (command == 'g' || command == 'c') {
            if (digitalRead(kHomeButtonPin) == LOW) {
                Serial.println("Tha nut BOOT truoc khi ra lenh chuyen dong");
                continue;
            }
            attachServo();
            setAngle(90);
            if (command == 'g') {
                direction = 1;
                state = ScanState::Centering;
                Serial.println("QUET: ve giua, sau do quet 0..180 do");
            } else {
                state = ScanState::Idle;
                Serial.println("GIU O GIUA 90 do");
            }
        } else if (command == '?') {
            help();
        }
    }
}

void processHomeButton() {
    const int raw = digitalRead(kHomeButtonPin);
    const uint32_t now = millis();
    if (raw != lastButtonRaw) {
        lastButtonRaw = raw;
        buttonChangedAtMs = now;
    }
    if (raw != buttonStable && uint32_t(now - buttonChangedAtMs) >= kButtonDebounceMs) {
        buttonStable = raw;
        if (buttonStable == LOW) returnToZero();
    }
}

uint32_t readEchoUs() {
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
    return pulseIn(Config::ULTRASONIC_ECHO_PIN, HIGH, kEchoTimeoutUs);
}

void takeReading() {
    const uint32_t echoUs = readEchoUs();
    if (echoUs == 0) {
        Serial.printf("Goc %3d do | ECHO timeout | khoang cach KHONG DO DUOC\n", angle);
    } else {
        const float cm = echoUs * 0.0343f / 2.0f;
        Serial.printf("Goc %3d do | ECHO %lu us | %.1f cm\n",
                      angle, static_cast<unsigned long>(echoUs), cm);
    }
}

void updateScan() {
    if (state == ScanState::Idle) return;
    const uint32_t now = millis();
    if (state == ScanState::Parking) {
        if (uint32_t(now - movedAtMs) < kCenterWaitMs) return;
        ledcWrite(kServoChannel, 0);
        ledcDetachPin(attachedPin());
        attached = false;
        attachedServo = -1;
        state = ScanState::Idle;
        Serial.println("SERVO DA VE 90 DO VA DA TAT XUNG");
        return;
    }
    if (state == ScanState::Centering) {
        if (uint32_t(now - movedAtMs) < kCenterWaitMs) return;
        setAngle(kMinAngle);
        state = ScanState::Settling;
        return;
    }
    if (uint32_t(now - movedAtMs) < kSettleMs) return;

    takeReading();
    if (angle >= kMaxAngle) direction = -1;
    else if (angle <= kMinAngle) direction = 1;
    setAngle(angle + direction * kStepDegrees);
}
}

void setup() {
    // Keep the trigger low; no servo PWM or ultrasonic pulse before an explicit command.
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
    pinMode(Config::ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(Config::ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(kHomeButtonPin, INPUT_PULLUP);
    lastButtonRaw = buttonStable = digitalRead(kHomeButtonPin);
    buttonChangedAtMs = millis();
    Serial.begin(115200);
    Serial.println("TEST SERVO QUET + HC-SR04; TRIG=GPIO12 ECHO=GPIO13");
    help();
}

void loop() {
    processSerial();
    processHomeButton();
    updateScan();
    delay(1);
}
