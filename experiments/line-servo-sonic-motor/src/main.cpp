#include <Arduino.h>

#include "Config.h"
#include "TestConfig.h"

namespace {

struct LineRaw {
  uint8_t left;
  uint8_t center;
  uint8_t right;
};

enum class ScanState : uint8_t {
  CENTERING,
  SETTLING,
  WAITING_ECHO,
  PARKING,
  STOPPED,
};

portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool echoWaiting = false;
volatile bool echoRiseSeen = false;
volatile bool echoReady = false;
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoDurationUs = 0;
volatile uint32_t echoTriggerUs = 0;

int lineActiveLevel = TestConfig::LINE_ACTIVE_LEVEL;
LineRaw calibrationCandidate{2, 2, 2};
uint32_t calibrationSinceMs = 0;

bool runEnabled = false;
bool servoAttached = false;
int servoAngle = 90;
int servoDirection = 1;
int measuredAngle = 90;
ScanState scanState = ScanState::CENTERING;
uint32_t scanStateSinceMs = 0;
uint32_t lastStatusMs = 0;
int bootLastRaw = HIGH;
int bootStable = HIGH;
uint32_t bootChangedAtMs = 0;

uint32_t servoPulseToDuty(uint16_t pulseUs) {
  const uint32_t periodUs = 1000000UL / Config::SERVO_PWM_HZ;
  const uint32_t maxDuty = (1UL << Config::SERVO_PWM_BITS) - 1UL;
  return (static_cast<uint32_t>(pulseUs) * maxDuty) / periodUs;
}

void attachServo() {
  if (servoAttached) {
    return;
  }
  ledcSetup(Config::SERVO_PWM_CHANNELS[TestConfig::SERVO_INDEX],
            Config::SERVO_PWM_HZ,
            Config::SERVO_PWM_BITS);
  ledcAttachPin(Config::SERVO_PINS[TestConfig::SERVO_INDEX],
                Config::SERVO_PWM_CHANNELS[TestConfig::SERVO_INDEX]);
  servoAttached = true;
}

void setServoAngle(int angle) {
  servoAngle = constrain(angle, 0, 180);
  const uint16_t pulseUs = map(servoAngle, 0, 180,
                               TestConfig::SERVO_MIN_US,
                               TestConfig::SERVO_MAX_US);
  ledcWrite(Config::SERVO_PWM_CHANNELS[TestConfig::SERVO_INDEX],
            servoPulseToDuty(pulseUs));
}

void detachServo() {
  if (!servoAttached) {
    return;
  }
  ledcWrite(Config::SERVO_PWM_CHANNELS[TestConfig::SERVO_INDEX], 0);
  ledcDetachPin(Config::SERVO_PINS[TestConfig::SERVO_INDEX]);
  servoAttached = false;
}

void IRAM_ATTR echoChangeIsr() {
  const bool level = digitalRead(Config::ULTRASONIC_ECHO_PIN);
  const uint32_t nowUs = micros();

  portENTER_CRITICAL_ISR(&echoMux);
  if (!echoWaiting) {
    portEXIT_CRITICAL_ISR(&echoMux);
    return;
  }

  if (level) {
    echoRiseUs = nowUs;
    echoRiseSeen = true;
  } else if (echoRiseSeen) {
    echoDurationUs = nowUs - echoRiseUs;
    echoReady = true;
    echoWaiting = false;
  }
  portEXIT_CRITICAL_ISR(&echoMux);
}

void cancelEcho() {
  portENTER_CRITICAL(&echoMux);
  echoWaiting = false;
  echoRiseSeen = false;
  echoReady = false;
  portEXIT_CRITICAL(&echoMux);
}

void startDistanceMeasurement(int angle) {
  measuredAngle = angle;

  portENTER_CRITICAL(&echoMux);
  echoWaiting = true;
  echoRiseSeen = false;
  echoReady = false;
  echoDurationUs = 0;
  echoTriggerUs = micros();
  portEXIT_CRITICAL(&echoMux);

  digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(Config::ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
}

int applyMotorTrim(bool left, int speed) {
  if (speed == 0) {
    return 0;
  }
  const int trim = left ? TestConfig::LEFT_PWM_TRIM : TestConfig::RIGHT_PWM_TRIM;
  const int magnitude = constrain(abs(speed) + trim, 0, 255);
  return speed < 0 ? -magnitude : magnitude;
}

bool takeDistanceResult(uint32_t &durationUs, bool &timedOut) {
  bool finished = false;
  timedOut = false;
  const uint32_t nowUs = micros();

  portENTER_CRITICAL(&echoMux);
  if (echoReady) {
    durationUs = echoDurationUs;
    echoReady = false;
    finished = true;
  } else if (echoWaiting &&
             static_cast<uint32_t>(nowUs - echoTriggerUs) >= TestConfig::ECHO_TIMEOUT_US) {
    echoWaiting = false;
    echoRiseSeen = false;
    timedOut = true;
    finished = true;
  }
  portEXIT_CRITICAL(&echoMux);
  return finished;
}

void setL298nSide(bool left, int speed) {
  const bool inverted = left ? TestConfig::LEFT_MOTOR_INVERTED
                             : TestConfig::RIGHT_MOTOR_INVERTED;
  int commanded = applyMotorTrim(left, constrain(speed, -255, 255));
  if (inverted) {
    commanded = -commanded;
  }

  const uint8_t pinA = left ? Config::LEFT_IN1 : Config::RIGHT_IN3;
  const uint8_t pinB = left ? Config::LEFT_IN2 : Config::RIGHT_IN4;
  const uint8_t channel = left ? Config::MOTOR_PWM_CHANNELS[0]
                               : Config::MOTOR_PWM_CHANNELS[1];

  digitalWrite(pinA, commanded > 0 ? HIGH : LOW);
  digitalWrite(pinB, commanded < 0 ? HIGH : LOW);
  ledcWrite(channel, abs(commanded));
}

void setBts7960Side(bool left, int speed) {
  const bool inverted = left ? TestConfig::LEFT_MOTOR_INVERTED
                             : TestConfig::RIGHT_MOTOR_INVERTED;
  int commanded = applyMotorTrim(left, constrain(speed, -255, 255));
  if (inverted) {
    commanded = -commanded;
  }

  const uint8_t forwardChannel = left ? Config::MOTOR_PWM_CHANNELS[0]
                                      : Config::MOTOR_PWM_CHANNELS[2];
  const uint8_t reverseChannel = left ? Config::MOTOR_PWM_CHANNELS[1]
                                      : Config::MOTOR_PWM_CHANNELS[3];
  ledcWrite(forwardChannel, commanded > 0 ? commanded : 0);
  ledcWrite(reverseChannel, commanded < 0 ? -commanded : 0);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (TestConfig::DRIVER == Config::MotorDriver::L298N) {
    setL298nSide(true, leftSpeed);
    setL298nSide(false, rightSpeed);
  } else if (TestConfig::DRIVER == Config::MotorDriver::BTS7960) {
    setBts7960Side(true, leftSpeed);
    setBts7960Side(false, rightSpeed);
  }
}

void stopMotors() {
  driveMotors(0, 0);
}

void beginMotors() {
  pinMode(Config::LEFT_ENA, OUTPUT);
  pinMode(Config::LEFT_IN1, OUTPUT);
  pinMode(Config::LEFT_IN2, OUTPUT);
  pinMode(Config::RIGHT_ENB, OUTPUT);
  pinMode(Config::RIGHT_IN3, OUTPUT);
  pinMode(Config::RIGHT_IN4, OUTPUT);

  digitalWrite(Config::LEFT_ENA, LOW);
  digitalWrite(Config::RIGHT_ENB, LOW);
  digitalWrite(Config::LEFT_IN1, LOW);
  digitalWrite(Config::LEFT_IN2, LOW);
  digitalWrite(Config::RIGHT_IN3, LOW);
  digitalWrite(Config::RIGHT_IN4, LOW);

  if (TestConfig::DRIVER == Config::MotorDriver::L298N) {
    ledcSetup(Config::MOTOR_PWM_CHANNELS[0],
              Config::MOTOR_PWM_HZ,
              Config::MOTOR_PWM_BITS);
    ledcSetup(Config::MOTOR_PWM_CHANNELS[1],
              Config::MOTOR_PWM_HZ,
              Config::MOTOR_PWM_BITS);
    ledcAttachPin(Config::LEFT_ENA, Config::MOTOR_PWM_CHANNELS[0]);
    ledcAttachPin(Config::RIGHT_ENB, Config::MOTOR_PWM_CHANNELS[1]);
  } else if (TestConfig::DRIVER == Config::MotorDriver::BTS7960) {
    digitalWrite(Config::LEFT_ENA, HIGH);
    digitalWrite(Config::RIGHT_ENB, HIGH);
    const uint8_t pins[] = {
        Config::LEFT_IN1, Config::LEFT_IN2,
        Config::RIGHT_IN3, Config::RIGHT_IN4};
    const uint8_t channels[] = {
        Config::MOTOR_PWM_CHANNELS[0],
        Config::MOTOR_PWM_CHANNELS[1],
        Config::MOTOR_PWM_CHANNELS[2],
        Config::MOTOR_PWM_CHANNELS[3]};
    for (size_t i = 0; i < 4; ++i) {
      ledcSetup(channels[i], Config::MOTOR_PWM_HZ,
                Config::MOTOR_PWM_BITS);
      ledcAttachPin(pins[i], channels[i]);
      ledcWrite(channels[i], 0);
    }
  }
  stopMotors();
}

LineRaw readPhysicalLineRaw() {
  return {
      static_cast<uint8_t>(digitalRead(Config::LINE_LEFT_PIN)),
      static_cast<uint8_t>(digitalRead(Config::LINE_CENTER_PIN)),
      static_cast<uint8_t>(digitalRead(Config::LINE_RIGHT_PIN)),
  };
}

LineRaw applyLineInputConfig(const LineRaw &physical) {
  if (!TestConfig::INVERT_LINE_INPUTS) {
    return physical;
  }
  return {
      static_cast<uint8_t>(1U - physical.left),
      static_cast<uint8_t>(1U - physical.center),
      static_cast<uint8_t>(1U - physical.right),
  };
}

void updateLineCalibration(const LineRaw &raw) {
  if (lineActiveLevel >= 0) {
    return;
  }

  // Vi tri khoi dong hop le: 2 cam bien ngoai giong nhau, cam bien giua nguoc lai.
  const bool centered = raw.left == raw.right && raw.center != raw.left;
  if (!centered) {
    calibrationCandidate = {2, 2, 2};
    calibrationSinceMs = 0;
    return;
  }

  if (raw.left != calibrationCandidate.left ||
      raw.center != calibrationCandidate.center ||
      raw.right != calibrationCandidate.right) {
    calibrationCandidate = raw;
    calibrationSinceMs = millis();
    return;
  }

  if (millis() - calibrationSinceMs >= TestConfig::LINE_CALIBRATION_STABLE_MS) {
    lineActiveLevel = raw.center;
    Serial.printf("Da nhan muc line den = %s (raw %u%u%u)\n",
                  lineActiveLevel ? "HIGH" : "LOW",
                  raw.left, raw.center, raw.right);
  }
}

uint8_t linePattern(const LineRaw &raw) {
  const uint8_t left = raw.left == lineActiveLevel ? 1 : 0;
  const uint8_t center = raw.center == lineActiveLevel ? 1 : 0;
  const uint8_t right = raw.right == lineActiveLevel ? 1 : 0;
  return static_cast<uint8_t>((left << 2) | (center << 1) | right);
}

const char *followLine(const LineRaw &raw) {
  if (!runEnabled) {
    stopMotors();
    return "TAM DUNG";
  }
  if (lineActiveLevel < 0) {
    stopMotors();
    return "CHO NHAN LINE";
  }

  switch (linePattern(raw)) {
    case 0b010:
      driveMotors(TestConfig::BASE_SPEED, TestConfig::BASE_SPEED);
      return "DI THANG";
    case 0b100:
    case 0b110:
      driveMotors(TestConfig::CORRECT_SPEED, 0);
      return "BANH TRAI CHAY - CUA PHAI VE LINE";
    case 0b001:
    case 0b011:
      driveMotors(0, TestConfig::CORRECT_SPEED);
      return "BANH PHAI CHAY - CUA TRAI VE LINE";
    case 0b111:
      driveMotors(TestConfig::BASE_SPEED, TestConfig::BASE_SPEED);
      return "GIAO LINE";
    case 0b000:
      stopMotors();
      return "MAT LINE - STOP";
    default:  // 101: mau khong ro rang, dung an toan.
      stopMotors();
      return "LINE MO HO - STOP";
  }
}

void advanceServo() {
  int nextAngle = servoAngle + servoDirection * TestConfig::SERVO_STEP_DEG;
  if (nextAngle >= 180) {
    nextAngle = 180;
    servoDirection = -1;
  } else if (nextAngle <= 0) {
    nextAngle = 0;
    servoDirection = 1;
  }
  setServoAngle(nextAngle);
  scanState = ScanState::SETTLING;
  scanStateSinceMs = millis();
}

void startAutomaticRun() {
  cancelEcho();
  attachServo();
  servoDirection = 1;
  setServoAngle(90);
  scanState = ScanState::CENTERING;
  scanStateSinceMs = millis();
  Serial.println("SERVO: ve 90 do, sau do quet 0-180. Motor cho nut BOOT.");
}

void stopAndPark() {
  runEnabled = false;
  stopMotors();
  cancelEcho();
  attachServo();
  setServoAngle(90);
  scanState = ScanState::PARKING;
  scanStateSinceMs = millis();
  Serial.println("STOP: motor dung, servo ve 90 do roi tat xung.");
}

void updateServoAndSonic() {
  const uint32_t nowMs = millis();

  switch (scanState) {
    case ScanState::CENTERING:
      if (nowMs - scanStateSinceMs >= TestConfig::SERVO_CENTER_WAIT_MS) {
        setServoAngle(0);
        scanState = ScanState::SETTLING;
        scanStateSinceMs = nowMs;
      }
      break;

    case ScanState::SETTLING:
      if (nowMs - scanStateSinceMs >= TestConfig::SERVO_SETTLE_MS) {
        startDistanceMeasurement(servoAngle);
        scanState = ScanState::WAITING_ECHO;
      }
      break;

    case ScanState::WAITING_ECHO: {
      uint32_t durationUs = 0;
      bool timedOut = false;
      if (takeDistanceResult(durationUs, timedOut)) {
        if (timedOut) {
          Serial.printf("Goc %3d do | ECHO timeout | KHONG DO DUOC\n", measuredAngle);
        } else {
          const float distanceCm = durationUs * 0.0343f / 2.0f;
          Serial.printf("Goc %3d do | %7.1f cm\n", measuredAngle, distanceCm);
        }
        advanceServo();
      }
      break;
    }

    case ScanState::PARKING:
      if (nowMs - scanStateSinceMs >= TestConfig::SERVO_CENTER_WAIT_MS) {
        detachServo();
        scanState = ScanState::STOPPED;
      }
      break;

    case ScanState::STOPPED:
      break;
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(tolower(Serial.read()));
    if (command == 's') {
      stopAndPark();
    } else if (command == 'g') {
      startAutomaticRun();
    } else if (command == '?') {
      Serial.println("s = dung motor, servo ve 90 roi tat; g = quet servo lai; BOOT = cho phep motor chay");
    }
  }
}

void updateBootButton() {
  const int raw = digitalRead(TestConfig::BOOT_BUTTON_PIN);
  const uint32_t nowMs = millis();

  if (raw != bootLastRaw) {
    bootLastRaw = raw;
    bootChangedAtMs = nowMs;
  }

  if (raw != bootStable &&
      nowMs - bootChangedAtMs >= TestConfig::BOOT_DEBOUNCE_MS) {
    bootStable = raw;
    if (bootStable == LOW) {
      runEnabled = !runEnabled;
      if (runEnabled) {
        if (scanState == ScanState::STOPPED || scanState == ScanState::PARKING) {
          startAutomaticRun();
        }
        Serial.println("BOOT: MOTOR BAT - xe chay khi co line.");
      } else {
        stopMotors();
        Serial.println("BOOT: MOTOR TAT - servo va HC-SR04 van quet.");
      }
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  delay(100);

  pinMode(Config::LINE_LEFT_PIN, INPUT);
  pinMode(Config::LINE_CENTER_PIN, INPUT);
  pinMode(Config::LINE_RIGHT_PIN, INPUT);

  pinMode(TestConfig::BOOT_BUTTON_PIN, INPUT_PULLUP);
  bootLastRaw = bootStable = digitalRead(TestConfig::BOOT_BUTTON_PIN);
  bootChangedAtMs = millis();

  pinMode(Config::ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
  pinMode(Config::ULTRASONIC_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(Config::ULTRASONIC_ECHO_PIN),
                  echoChangeIsr, CHANGE);

  beginMotors();

  Serial.println();
  Serial.println("===== LINE + MOTOR + SERVO + HC-SR04 TEST =====");
  Serial.printf("Line L/C/R: GPIO%d/%d/%d | Servo: GPIO%d | Sonic TRIG/ECHO: GPIO%d/%d\n",
                Config::LINE_LEFT_PIN,
                Config::LINE_CENTER_PIN,
                Config::LINE_RIGHT_PIN,
                Config::SERVO_PINS[TestConfig::SERVO_INDEX],
                Config::ULTRASONIC_TRIG_PIN,
                Config::ULTRASONIC_ECHO_PIN);
  Serial.println("Cam bien line doc truc tiep GPIO; line den = 1, nen trang = 0.");
  Serial.println("Motor dang TAT. Moi lan bam BOOT se BAT/TAT motor.");
  Serial.println("Lenh: s=dung khan cap, g=quet servo lai, ?=huong dan");
  startAutomaticRun();
}

void loop() {
  handleSerial();
  updateBootButton();
  updateServoAndSonic();

  const LineRaw physicalRaw = readPhysicalLineRaw();
  const LineRaw raw = applyLineInputConfig(physicalRaw);
  updateLineCalibration(raw);
  const char *action = followLine(raw);

  const uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= TestConfig::STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    if (lineActiveLevel < 0) {
      Serial.printf("LINE GPIO=%u%u%u logic=%u%u%u | %s\n",
                    physicalRaw.left, physicalRaw.center, physicalRaw.right,
                    raw.left, raw.center, raw.right, action);
    } else {
      Serial.printf("LINE GPIO=%u%u%u logic=%u%u%u pattern=%u%u%u | %s\n",
                    physicalRaw.left, physicalRaw.center, physicalRaw.right,
                    raw.left, raw.center, raw.right,
                    raw.left == lineActiveLevel,
                    raw.center == lineActiveLevel,
                    raw.right == lineActiveLevel,
                    action);
    }
  }
}

