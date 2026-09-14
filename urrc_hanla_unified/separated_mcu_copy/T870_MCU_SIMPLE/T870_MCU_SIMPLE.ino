/*
 * T870_MCU_CMD_BRIDGE v1
 * 역할: 상위 ROS의 cmd_drive / cmd_wheel을 그대로 실행하는 최소 MCU.
 *
 * 명령
 *   D,<signed_pwm> : +전진 / -후진 / 0정지
 *   W,<deg>        : 절대 조향각, +LEFT / -RIGHT, ±22deg
 *   X              : 구동+조향 즉시 출력 0
 *   ARM/DISARM     : 출력 허용/차단
 *   CFG,...        : 런타임 설정
 *   Q              : 상태 1회 출력
 *
 * 조향 기준
 *   center ADC = 484
 *   18 ADC count / deg
 *   +deg = LEFT, -deg = RIGHT
 *
 * 구동 엔코더
 *   ENC_A = Arduino Mega D2
 *   A상 RISING만 카운트, 200us debounce
 *   D3(B상)는 배선 호환용이며 카운트에는 사용하지 않음
 */

#include <Arduino.h>
#include <math.h>

// ---------- 고정 배선 ----------
constexpr uint8_t PWM_DRIVE_FRONT = 9;
constexpr uint8_t DIR_DRIVE_FRONT = 10;
constexpr uint8_t PWM_DRIVE_REAR  = 7;
constexpr uint8_t DIR_DRIVE_REAR  = 8;
constexpr uint8_t PWM_STEER       = 11;
constexpr uint8_t DIR_STEER       = 12;
constexpr uint8_t POT_STEER       = A0;

// 구동 엔코더: 실차에서 값이 확인된 A상 D2만 카운트한다.
// B상 D3는 배선 호환용이며 현재 카운트/방향판정에는 사용하지 않는다.
constexpr uint8_t ENC_A = 2;
constexpr uint8_t ENC_B = 3;  // 미사용, 배선 호환용
constexpr unsigned long ENCODER_DEBOUNCE_US = 200;

// ---------- 런타임 설정 기본값 ----------
int steerCenterAdc = 484;
float steerCountsPerDeg = 18.0f;
int maxSteerDeg = 22;
int steerPwm = 130;
int steerToleranceAdc = 4;
unsigned long steerSettleMs = 250;
int steerFineBandAdc = 50;
unsigned long steerFineOnMs = 30;
unsigned long steerFineCycleMs = 90;
unsigned long steerTimeoutMs = 5000;
unsigned long driveTimeoutMs = 700;
unsigned long statusPeriodMs = 200;

// 1=HIGH가 해당 정방향, 0=LOW가 해당 정방향.
int frontForwardLevel = 0;
int rearForwardLevel = 0;
int steerLeftLevel = 1;

// ---------- 상태 ----------
bool armed = false;
int driveSignedPwm = 0;
unsigned long lastDriveCmdMs = 0;

bool steerActive = false;
int steerTargetDeg = 0;
int steerTargetAdc = 484;
unsigned long steerStartedMs = 0;
unsigned long steerInsideSinceMs = 0;

unsigned long lastStatusMs = 0;

char rxBuf[96];
uint8_t rxLen = 0;

volatile long encoderCountA = 0;
volatile unsigned long lastEncoderUs = 0;

void encoderISR() {
  const unsigned long now = micros();
  if (now - lastEncoderUs >= ENCODER_DEBOUNCE_US) {
    lastEncoderUs = now;
    encoderCountA++;
  }
}

void stopDrive() {
  analogWrite(PWM_DRIVE_FRONT, 0);
  analogWrite(PWM_DRIVE_REAR, 0);
  driveSignedPwm = 0;
}

// 모터 출력만 끄고 steerActive는 유지할 수 있도록 분리한다.
void stopSteerMotorOnly() {
  analogWrite(PWM_STEER, 0);
}

void cancelSteer() {
  stopSteerMotorOnly();
  steerActive = false;
  steerInsideSinceMs = 0;
}

void stopAll() {
  stopDrive();
  cancelSteer();
}

void setDriveSignedPwm(int signedPwm) {
  signedPwm = constrain(signedPwm, -255, 255);

  if (!armed || signedPwm == 0) {
    stopDrive();
    lastDriveCmdMs = millis();
    return;
  }

  const bool forward = signedPwm > 0;
  const int pwm = abs(signedPwm);

  const int frontLevel = forward ? frontForwardLevel : !frontForwardLevel;
  const int rearLevel  = forward ? rearForwardLevel  : !rearForwardLevel;

  digitalWrite(DIR_DRIVE_FRONT, frontLevel ? HIGH : LOW);
  digitalWrite(DIR_DRIVE_REAR,  rearLevel  ? HIGH : LOW);
  analogWrite(PWM_DRIVE_FRONT, pwm);
  analogWrite(PWM_DRIVE_REAR,  pwm);

  driveSignedPwm = forward ? pwm : -pwm;
  lastDriveCmdMs = millis();
}

void setSteerDeg(int deg) {
  if (!armed) return;

  deg = constrain(deg, -maxSteerDeg, maxSteerDeg);
  steerTargetDeg = deg;
  steerTargetAdc = steerCenterAdc + (int)lroundf((float)deg * steerCountsPerDeg);
  steerTargetAdc = constrain(steerTargetAdc, 20, 1003);

  steerStartedMs = millis();
  steerInsideSinceMs = 0;
  steerActive = true;
}

void updateSteer() {
  if (!steerActive) return;

  const unsigned long now = millis();
  const int adc = analogRead(POT_STEER);
  const int err = steerTargetAdc - adc;
  const int ae = abs(err);

  // 핵심 수정:
  // 목표 범위에 한 번 들어왔다고 즉시 완료하지 않는다.
  // PWM을 끈 뒤 일정 시간 계속 범위 안에 머무는지 확인한다.
  // 관성/백래시로 다시 벗어나면 자동으로 재보정한다.
  if (ae <= steerToleranceAdc) {
    stopSteerMotorOnly();

    if (steerInsideSinceMs == 0) {
      steerInsideSinceMs = now;
    }

    if (now - steerInsideSinceMs >= steerSettleMs) {
      steerActive = false;
      Serial.print(F("EVT,STEER_REACHED,"));
      Serial.print(adc);
      Serial.print(',');
      Serial.println(steerTargetAdc);
    }
    return;
  }

  // 목표에서 벗어나 있으면 settle 판정을 다시 시작한다.
  steerInsideSinceMs = 0;

  if (now - steerStartedMs > steerTimeoutMs) {
    cancelSteer();
    Serial.print(F("EVT,STEER_TIMEOUT,"));
    Serial.print(adc);
    Serial.print(',');
    Serial.println(steerTargetAdc);
    return;
  }

  if (adc <= 15 || adc >= 1008) {
    cancelSteer();
    Serial.println(F("EVT,STEER_ADC_LIMIT"));
    return;
  }

  // ADC 증가=LEFT / ADC 감소=RIGHT
  const bool needLeft = err > 0;
  const int level = needLeft ? steerLeftLevel : !steerLeftLevel;
  digitalWrite(DIR_STEER, level ? HIGH : LOW);

  // 목표 근처에서는 130 PWM을 계속 때리지 않고 짧게 펄스해서
  // 관성으로 중앙을 지나쳐 버리는 현상을 줄인다.
  if (ae <= steerFineBandAdc) {
    const unsigned long phase = (now - steerStartedMs) % steerFineCycleMs;
    analogWrite(PWM_STEER, phase < steerFineOnMs ? steerPwm : 0);
  } else {
    analogWrite(PWM_STEER, steerPwm);
  }
}

void publishStatus() {
  const int adc = analogRead(POT_STEER);
  long encA;
  noInterrupts();
  encA = encoderCountA;
  interrupts();

  // STAT,adc,target_adc,target_deg,drive_signed_pwm,steer_active,armed,encA
  Serial.print(F("STAT,"));
  Serial.print(adc); Serial.print(',');
  Serial.print(steerTargetAdc); Serial.print(',');
  Serial.print(steerTargetDeg); Serial.print(',');
  Serial.print(driveSignedPwm); Serial.print(',');
  Serial.print(steerActive ? 1 : 0); Serial.print(',');
  Serial.print(armed ? 1 : 0); Serial.print(',');
  Serial.println(encA);
}

bool startsWith(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

void processCfg(char* cmd) {
  char* save = nullptr;
  strtok_r(cmd, ",", &save); // CFG
  char* key = strtok_r(nullptr, ",", &save);
  char* val = strtok_r(nullptr, ",", &save);

  if (!key || !val) {
    Serial.println(F("ERR,CFG"));
    return;
  }

  if (strcmp(key, "CENTER") == 0) {
    steerCenterAdc = constrain(atoi(val), 20, 1003);
  } else if (strcmp(key, "COUNTS_PER_DEG") == 0) {
    const float v = atof(val);
    if (v > 0.1f && v < 100.0f) steerCountsPerDeg = v;
  } else if (strcmp(key, "MAX_DEG") == 0) {
    maxSteerDeg = constrain(atoi(val), 1, 30);
  } else if (strcmp(key, "STEER_PWM") == 0) {
    steerPwm = constrain(atoi(val), 0, 255);
  } else if (strcmp(key, "STEER_TOL") == 0) {
    steerToleranceAdc = constrain(atoi(val), 1, 30);
  } else if (strcmp(key, "STEER_SETTLE_MS") == 0) {
    steerSettleMs = constrain(atol(val), 50L, 2000L);
  } else if (strcmp(key, "STEER_FINE_BAND") == 0) {
    steerFineBandAdc = constrain(atoi(val), 5, 200);
  } else if (strcmp(key, "STEER_FINE_ON_MS") == 0) {
    steerFineOnMs = constrain(atol(val), 5L, 200L);
  } else if (strcmp(key, "STEER_FINE_CYCLE_MS") == 0) {
    steerFineCycleMs = constrain(atol(val), 10L, 500L);
    if (steerFineOnMs >= steerFineCycleMs) steerFineOnMs = steerFineCycleMs - 1;
  } else if (strcmp(key, "STEER_TIMEOUT_MS") == 0) {
    steerTimeoutMs = constrain(atol(val), 100L, 10000L);
  } else if (strcmp(key, "DRIVE_TIMEOUT_MS") == 0) {
    driveTimeoutMs = constrain(atol(val), 100L, 5000L);
  } else if (strcmp(key, "STATUS_MS") == 0) {
    statusPeriodMs = constrain(atol(val), 50L, 5000L);
  } else if (strcmp(key, "FRONT_FWD") == 0) {
    frontForwardLevel = atoi(val) ? 1 : 0;
  } else if (strcmp(key, "REAR_FWD") == 0) {
    rearForwardLevel = atoi(val) ? 1 : 0;
  } else if (strcmp(key, "STEER_LEFT") == 0) {
    steerLeftLevel = atoi(val) ? 1 : 0;
  } else {
    Serial.println(F("ERR,CFG_KEY"));
    return;
  }

  Serial.print(F("OK,CFG,"));
  Serial.println(key);
}

void processCommand(char* cmd) {
  while (*cmd == ' ') cmd++;
  size_t n = strlen(cmd);
  while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\r' || cmd[n - 1] == '\n')) {
    cmd[--n] = '\0';
  }
  if (n == 0) return;

  if (strcmp(cmd, "X") == 0) {
    stopAll();
    Serial.println(F("OK,X"));
    return;
  }

  if (strcmp(cmd, "ARM") == 0) {
    stopAll();
    armed = true;
    lastDriveCmdMs = millis();
    Serial.println(F("OK,ARM"));
    return;
  }

  if (strcmp(cmd, "DISARM") == 0) {
    stopAll();
    armed = false;
    Serial.println(F("OK,DISARM"));
    return;
  }

  if (strcmp(cmd, "Q") == 0) {
    publishStatus();
    return;
  }

  if (startsWith(cmd, "CFG,")) {
    processCfg(cmd);
    return;
  }

  if (startsWith(cmd, "D,")) {
    if (!armed) {
      Serial.println(F("ERR,NOT_ARMED"));
      return;
    }
    setDriveSignedPwm(atoi(cmd + 2));
    return;
  }

  if (startsWith(cmd, "W,")) {
    if (!armed) {
      Serial.println(F("ERR,NOT_ARMED"));
      return;
    }
    setSteerDeg(atoi(cmd + 2));
    return;
  }

  Serial.println(F("ERR,UNKNOWN"));
}

void readSerialLines() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      processCommand(rxBuf);
      rxLen = 0;
    } else if (c != '\r') {
      if (rxLen < sizeof(rxBuf) - 1) {
        rxBuf[rxLen++] = c;
      } else {
        rxLen = 0;
        Serial.println(F("ERR,RX_OVERFLOW"));
      }
    }
  }
}

void setup() {
  pinMode(PWM_DRIVE_FRONT, OUTPUT);
  pinMode(DIR_DRIVE_FRONT, OUTPUT);
  pinMode(PWM_DRIVE_REAR, OUTPUT);
  pinMode(DIR_DRIVE_REAR, OUTPUT);
  pinMode(PWM_STEER, OUTPUT);
  pinMode(DIR_STEER, OUTPUT);
  pinMode(POT_STEER, INPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  stopAll();
  armed = false;

  lastEncoderUs = micros();
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.begin(115200);
  delay(50);
  Serial.print(F("READY,"));
  Serial.print(steerCenterAdc);
  Serial.println(F(",ENC_A_ONLY,D2,RISING,DEBOUNCE_US=200"));
}

void loop() {
  readSerialLines();

  // 상위 drive 명령이 끊기면 자동 정지. MCU가 스스로 단수를 만들지 않는다.
  if (armed && driveSignedPwm != 0 && (millis() - lastDriveCmdMs > driveTimeoutMs)) {
    stopDrive();
    Serial.println(F("EVT,DRIVE_TIMEOUT"));
  }

  updateSteer();

  const unsigned long now = millis();
  if (now - lastStatusMs >= statusPeriodMs) {
    lastStatusMs = now;
    publishStatus();
  }
}
