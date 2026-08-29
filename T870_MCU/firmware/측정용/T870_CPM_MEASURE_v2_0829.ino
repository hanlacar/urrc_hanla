// ==========================================================
//  T870 counts_per_meter 측정 전용 스케치 v2  (2026-08-29)
//  v2: 후진 측정(B) + 차 되돌리기용 조그(J/K) 추가
//
//  ROS 없이 아두이노 단독으로 잰다. 시리얼 모니터만 있으면 된다.
//  중간에 끼어드는 노드가 없어 결과가 깨끗하다.
//
//  ★ 측정이 끝나면 반드시 T870_MCU_v33.ino 로 되돌릴 것.
//    이 스케치에는 워치독도, 중재도, 안전 상태머신도 없다.
//
// ----------------------------------------------------------
//  쓰는 법
//    1. 아두이노 IDE 로 업로드 (보드: Arduino Mega 2560)
//    2. 시리얼 모니터 열기 — 115200 baud, 줄 끝: "새 줄"
//    3. 바퀴를 직진으로 맞춘다 (L / R 로 미세조정 가능)
//    4. 출발선을 바닥에 표시하고 차를 세운다
//    5. G 입력 → 10초 전진 후 자동 정지
//    6. 완전히 멈추면 총 카운트가 출력된다
//    7. 줄자로 출발선~멈춘 자리를 재고  D6.787  처럼 입력
//       → counts_per_meter 를 계산해 준다
//
//  명령
//    G          전진 측정 (기본 2단, 10초)
//    B          후진 측정 (같은 절차, 방향만 반대)
//    S          즉시 정지
//    J<ms>      전진 조그 — 차 위치 되돌릴 때.  예: J800  (기본 800ms)
//    K<ms>      후진 조그 — 출발선으로 되돌릴 때. 예: K1500
//    1 2 3      단수 선택 (기본 2)
//    T<초>      주행 시간 변경.  예: T10
//    L / R      조향 좌 / 우 120ms 미세 이동 (직진 맞출 때만)
//    Z          카운터 0으로
//    D<거리>    실측 거리[m] 를 넣어 계산.  예: D6.787
//    ?          도움말
//
//  ※ 엔코더는 A상만 세므로 방향을 구분하지 못한다.
//    후진해도 카운트는 그냥 증가한다. 크기만 쓰므로 측정에는 문제없다.
//    조그(J/K)는 카운트에 영향을 주니 측정 직전에 Z 로 초기화할 것.
//
//  ⚠ 안전
//    - 앞 공간을 넉넉히 확보할 것 (2단 10초면 5~10m 는 간다)
//    - 브레이크가 없어 정지 명령 후에도 굴러간다
//    - 주행 중 아무 키나 누르면 즉시 정지한다
//    - E-Stop(D24)이 눌리면 즉시 정지한다
// ==========================================================

#include <Arduino.h>

// ---------- 핀 (v33 과 동일) ----------
constexpr uint8_t PWM_DRIVE_FRONT = 9;
constexpr uint8_t DIR_DRIVE_FRONT = 10;
constexpr uint8_t PWM_DRIVE_REAR  = 7;
constexpr uint8_t DIR_DRIVE_REAR  = 8;
constexpr uint8_t PWM_STEER       = 11;
constexpr uint8_t DIR_STEER       = 12;
constexpr uint8_t ENC_A           = 2;
constexpr uint8_t ESTOP_PIN       = 24;

// ---------- 극성 (v33 과 동일) ----------
constexpr uint8_t DRIVE_FRONT_FWD = LOW;
constexpr uint8_t DRIVE_FRONT_REV = HIGH;
constexpr uint8_t DRIVE_REAR_FWD  = HIGH;
constexpr uint8_t DRIVE_REAR_REV  = LOW;
constexpr uint8_t STEER_DIR_LEFT  = HIGH;
constexpr uint8_t STEER_DIR_RIGHT = LOW;
constexpr uint8_t STEER_PWM_LEVEL = 130;

// ---------- 구동 (v33 과 동일) ----------
constexpr int DRIVE_PWM_STAGE_1 = 50;
constexpr int DRIVE_PWM_STAGE_2 = 100;
constexpr int DRIVE_PWM_STAGE_3 = 150;
constexpr uint8_t       ACCEL_STEP   = 3;    // 50ms 당
constexpr uint8_t       DECEL_STEP   = 10;
constexpr unsigned long RAMP_MS      = 50;

// ---------- 엔코더 (v33 과 동일) ----------
constexpr uint32_t ENCODER_DEBOUNCE_US = 200;
volatile long     encoderPulse  = 0;
volatile uint32_t encoderLastUs = 0;

// ---------- 참고값 ----------
constexpr float WHEEL_CIRC_M = 0.817f;   // 바퀴 둘레 (검증 대상)
constexpr float CPM_CURRENT  = 533.1f;   // 지금 yaml 에 들어 있는 값

// ---------- 상태 ----------
enum Phase : uint8_t { IDLE, ACCEL, HOLD, DECEL, COAST, REPORT };
Phase phase = IDLE;

int  stageSel     = 2;
bool runForward   = true;    // 이번 주행 방향
bool doReport     = true;    // 조그면 결과를 찍지 않는다
int  targetPwm    = 0;
int  currentPwm   = 0;
unsigned long runMs        = 10000;   // 측정 주행 시간 (T 로 변경)
unsigned long runMsActive  = 10000;   // 이번 주행에 실제로 쓸 시간
unsigned long phaseStartMs = 0;
unsigned long lastRampMs   = 0;
unsigned long lastTickMs   = 0;
unsigned long coastStartMs = 0;
long startCount = 0;
long lastCoastCount = 0;
long resultCount = 0;
unsigned long resultMs = 0;

char rx[24];
uint8_t rxLen = 0;

// ==========================================================
void encoderIsr()
{
  uint32_t nowUs = micros();
  if (nowUs - encoderLastUs < ENCODER_DEBOUNCE_US) return;
  encoderLastUs = nowUs;
  encoderPulse++;
}

long readCount()
{
  long v;
  noInterrupts();
  v = encoderPulse;
  interrupts();
  return v;
}

int stagePwm(int s)
{
  if (s == 1) return DRIVE_PWM_STAGE_1;
  if (s == 3) return DRIVE_PWM_STAGE_3;
  return DRIVE_PWM_STAGE_2;
}

void applyPwm(int pwm)
{
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  currentPwm = pwm;
  analogWrite(PWM_DRIVE_FRONT, pwm);
  analogWrite(PWM_DRIVE_REAR, pwm);
}

void setDirection(bool forward)
{
  digitalWrite(DIR_DRIVE_FRONT, forward ? DRIVE_FRONT_FWD : DRIVE_FRONT_REV);
  digitalWrite(DIR_DRIVE_REAR,  forward ? DRIVE_REAR_FWD  : DRIVE_REAR_REV);
}

void hardStop(const __FlashStringHelper *why)
{
  applyPwm(0);
  targetPwm = 0;
  analogWrite(PWM_STEER, 0);
  if (phase != IDLE && phase != REPORT) {
    Serial.print(F("STOP,"));
    Serial.println(why);
  }
  phase = IDLE;
}

bool estopActive()
{
  return digitalRead(ESTOP_PIN) == HIGH;   // NC 접점: 정상 LOW
}

void nudgeSteer(bool left)
{
  if (phase != IDLE) {
    Serial.println(F("ERR,주행 중에는 조향 조작 금지"));
    return;
  }
  digitalWrite(DIR_STEER, left ? STEER_DIR_LEFT : STEER_DIR_RIGHT);
  analogWrite(PWM_STEER, STEER_PWM_LEVEL);
  delay(120);
  analogWrite(PWM_STEER, 0);
  Serial.print(F("STEER,"));
  Serial.println(left ? F("LEFT_120ms") : F("RIGHT_120ms"));
}

void printHelp()
{
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F(" G        전진 측정"));
  Serial.println(F(" B        후진 측정"));
  Serial.println(F(" J<ms>    전진 조그 (기본 800ms).  예 J800"));
  Serial.println(F(" K<ms>    후진 조그 (기본 800ms).  예 K1500"));
  Serial.println(F(" S        즉시 정지"));
  Serial.println(F(" 1 2 3    단수 선택"));
  Serial.println(F(" T<초>    주행 시간.  예 T10"));
  Serial.println(F(" L / R    조향 좌/우 120ms (정지 중에만)"));
  Serial.println(F(" Z        카운터 0"));
  Serial.println(F(" D<거리>  실측 거리[m] 넣어 계산.  예 D6.787"));
  Serial.println(F(" ?        도움말"));
  Serial.println(F("--------------------------------------------------"));
  Serial.print(F(" 현재 설정: "));
  Serial.print(stageSel);
  Serial.print(F("단 (PWM "));
  Serial.print(stagePwm(stageSel));
  Serial.print(F("),  "));
  Serial.print(runMs / 1000.0, 1);
  Serial.println(F("초"));
}

void startRun(bool forward, unsigned long durationMs, bool report)
{
  if (phase != IDLE) {
    Serial.println(F("ERR,이미 진행 중"));
    return;
  }
  if (estopActive()) {
    Serial.println(F("ERR,E-Stop 이 눌려 있다"));
    return;
  }
  runForward   = forward;
  doReport     = report;
  runMsActive  = durationMs;
  startCount   = readCount();
  targetPwm    = stagePwm(stageSel);
  phase        = ACCEL;
  phaseStartMs = millis();
  lastRampMs   = phaseStartMs;
  lastTickMs   = phaseStartMs;
  setDirection(forward);
  applyPwm(0);

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.print(report ? F("GO  ") : F("JOG "));
  Serial.print(forward ? F("전진  ") : F("후진  "));
  Serial.print(stageSel);
  Serial.print(F("단(PWM "));
  Serial.print(targetPwm);
  Serial.print(F("),  "));
  Serial.print(durationMs / 1000.0, 2);
  Serial.print(F("초,  시작 카운트 "));
  Serial.println(startCount);
  Serial.println(F("=================================================="));
}

void report(long counts, unsigned long ms)
{
  resultCount = counts;
  resultMs    = ms;
  Serial.println();
  Serial.println(F("=================== 결과 ==================="));
  Serial.print(F("  방향        "));
  Serial.println(runForward ? F("전진") : F("후진"));
  Serial.print(F("  단수        "));
  Serial.print(stageSel);
  Serial.print(F("단 (PWM "));
  Serial.print(stagePwm(stageSel));
  Serial.println(F(")"));
  Serial.print(F("  총 카운트   "));
  Serial.println(counts);
  Serial.print(F("  주행 시간   "));
  Serial.print(ms / 1000.0, 2);
  Serial.println(F(" s"));
  Serial.println();
  Serial.println(F("  줄자로 출발선~멈춘 자리를 재세요."));
  Serial.println(F("  그 값을 D 로 입력하면 계산합니다.  예:  D6.787"));
  Serial.println(F("============================================"));
  Serial.println();
}

void compute(float meters)
{
  if (resultCount <= 0) {
    Serial.println(F("ERR,먼저 G 로 측정하세요"));
    return;
  }
  if (meters <= 0.01f) {
    Serial.println(F("ERR,거리가 0 이다. 미터 단위로 입력.  예 D6.787"));
    return;
  }
  float cpm  = resultCount / meters;
  float mps  = meters / (resultMs / 1000.0f);
  float perRev = cpm * WHEEL_CIRC_M;

  Serial.println();
  Serial.println(F("================ 계산 결과 ================="));
  Serial.print(F("  실측 거리          "));
  Serial.print(meters, 3);
  Serial.println(F(" m"));
  Serial.print(F("  counts_per_meter   "));
  Serial.println(cpm, 1);
  Serial.print(F("  현재 설정값        "));
  Serial.println(CPM_CURRENT, 1);
  Serial.print(F("  배율               "));
  Serial.print(cpm / CPM_CURRENT, 2);
  Serial.println(F(" 배"));
  Serial.println();
  Serial.print(F("  1 카운트           "));
  Serial.print(1000.0f / cpm, 3);
  Serial.println(F(" mm"));
  Serial.print(F("  바퀴 1회전         "));
  Serial.print(perRev, 0);
  Serial.print(F(" 카운트  (둘레 "));
  Serial.print(WHEEL_CIRC_M, 3);
  Serial.println(F("m 가정)"));
  Serial.print(F("  이번 평균 속도     "));
  Serial.print(mps, 3);
  Serial.println(F(" m/s"));
  Serial.print(F("  바퀴 회전수        "));
  Serial.print(meters / WHEEL_CIRC_M, 2);
  Serial.println(F(" 바퀴  (교차검증용)"));
  Serial.println();
  Serial.println(F("  yaml 에 넣을 값:"));
  Serial.print(F("      counts_per_meter: "));
  Serial.println(cpm, 1);
  Serial.println(F("============================================"));
  Serial.println();
}

void handleCommand(char *cmd)
{
  if (cmd[0] == '\0') return;

  // 주행 중에는 어떤 입력이든 즉시 정지
  if (phase == ACCEL || phase == HOLD) {
    hardStop(F("USER"));
    phase = DECEL;
    phaseStartMs = millis();
    return;
  }

  char c = toupper(cmd[0]);

  if (c == 'G')       { startRun(true,  runMs, true); }
  else if (c == 'B')  { startRun(false, runMs, true); }
  else if (c == 'J' || c == 'K') {
    long ms = atol(cmd + 1);
    if (ms <= 0) ms = 800;                    // 기본 800ms
    if (ms > 5000) ms = 5000;                 // 안전 상한
    startRun(c == 'J', (unsigned long)ms, false);
  }
  else if (c == 'S')  { hardStop(F("USER")); }
  else if (c == '?')  { printHelp(); }
  else if (c == 'Z')  {
    noInterrupts(); encoderPulse = 0; interrupts();
    resultCount = 0;
    Serial.println(F("COUNTER,0"));
  }
  else if (c == '1' || c == '2' || c == '3') {
    stageSel = c - '0';
    Serial.print(F("STAGE,"));
    Serial.print(stageSel);
    Serial.print(F(",PWM,"));
    Serial.println(stagePwm(stageSel));
  }
  else if (c == 'T') {
    float sec = atof(cmd + 1);
    if (sec < 1.0f || sec > 60.0f) {
      Serial.println(F("ERR,1~60 초 사이"));
    } else {
      runMs = (unsigned long)(sec * 1000.0f);
      Serial.print(F("TIME,"));
      Serial.print(sec, 1);
      Serial.println(F("s"));
    }
  }
  else if (c == 'L')  { nudgeSteer(true); }
  else if (c == 'R')  { nudgeSteer(false); }
  else if (c == 'D')  { compute(atof(cmd + 1)); }
  else {
    Serial.print(F("ERR,알 수 없는 명령: "));
    Serial.println(cmd);
  }
}

void pollSerial()
{
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rx[rxLen] = '\0';
      handleCommand(rx);
      rxLen = 0;
      continue;
    }
    if (rxLen < sizeof(rx) - 1) rx[rxLen++] = ch;
  }
}

// ==========================================================
void setup()
{
  pinMode(PWM_DRIVE_FRONT, OUTPUT);
  pinMode(DIR_DRIVE_FRONT, OUTPUT);
  pinMode(PWM_DRIVE_REAR,  OUTPUT);
  pinMode(DIR_DRIVE_REAR,  OUTPUT);
  pinMode(PWM_STEER,       OUTPUT);
  pinMode(DIR_STEER,       OUTPUT);
  pinMode(ENC_A,           INPUT_PULLUP);
  pinMode(ESTOP_PIN,       INPUT_PULLUP);

  applyPwm(0);
  analogWrite(PWM_STEER, 0);
  setDirection(true);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderIsr, RISING);

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("  T870 counts_per_meter 측정 스케치"));
  Serial.println(F("  ★ 측정 후 반드시 T870_MCU_v33.ino 로 되돌릴 것"));
  Serial.println(F("=================================================="));
  printHelp();
  Serial.println(F("바퀴를 직진으로 맞추고, 출발선에 세운 뒤 G 를 입력하세요."));
}

void loop()
{
  unsigned long now = millis();
  pollSerial();

  // ---- E-Stop 감시 ----
  if (estopActive() && phase != IDLE && phase != REPORT) {
    hardStop(F("ESTOP"));
    return;
  }

  switch (phase) {

    case ACCEL:
      if (now - lastRampMs >= RAMP_MS) {
        lastRampMs = now;
        int p = currentPwm + ACCEL_STEP;
        if (p >= targetPwm) { p = targetPwm; phase = HOLD; }
        applyPwm(p);
      }
      // 가속 중에도 전체 시간은 흐른다
      if (now - phaseStartMs >= runMsActive) { phase = DECEL; }
      break;

    case HOLD:
      if (now - phaseStartMs >= runMsActive) {
        phase = DECEL;
        lastRampMs = now;
        Serial.println(F("RAMP_DOWN"));
      }
      break;

    case DECEL:
      if (now - lastRampMs >= RAMP_MS) {
        lastRampMs = now;
        int p = currentPwm - DECEL_STEP;
        if (p <= 0) {
          p = 0;
          phase = COAST;
          coastStartMs = now;
          lastCoastCount = readCount();
          Serial.println(F("COASTING... (완전히 멈출 때까지 기다립니다)"));
        }
        applyPwm(p);
      }
      break;

    case COAST:
      // 0.5초 동안 카운트가 안 늘면 멈춘 것으로 본다
      if (now - lastTickMs >= 500) {
        lastTickMs = now;
        long c = readCount();
        if (c == lastCoastCount) {
          phase = REPORT;
        }
        lastCoastCount = c;
      }
      // 최대 10초까지만 기다린다
      if (now - coastStartMs > 10000) phase = REPORT;
      break;

    case REPORT:
      if (doReport) {
        report(readCount() - startCount, now - phaseStartMs);
      } else {
        Serial.print(F("JOG_DONE, 이동 카운트 "));
        Serial.println(readCount() - startCount);
      }
      phase = IDLE;
      break;

    case IDLE:
    default:
      break;
  }

  // ---- 진행 중 1초마다 상태 출력 ----
  if ((phase == ACCEL || phase == HOLD) && now - lastTickMs >= 1000) {
    lastTickMs = now;
    Serial.print(F("  t="));
    Serial.print((now - phaseStartMs) / 1000.0, 1);
    Serial.print(F("s  pwm="));
    Serial.print(currentPwm);
    Serial.print(F("  count="));
    Serial.println(readCount() - startCount);
  }
}
