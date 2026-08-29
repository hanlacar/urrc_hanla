// ==========================================================
//  T870 하위제어기 v30
//
//  BROON T870 개조 자율주행 차량 (24V, Ackermann)
//  Arduino Mega 2560 / 115200 baud / 외부 라이브러리 없음
//
// ----------------------------------------------------------
//  [실측 확정값]  2026-08 기준
//    바퀴 지름      0.26 m   (둘레 0.817 m)
//    축거           0.73 m
//    최대 조향각    27도 (좌우 대칭)
//    최소 선회반경  1.43 m   (= 0.73 / tan27)
//    2.00 (PWM 50)  0.229 m/s
//    3.00 (PWM 100) 0.526 m/s
//    4.00 (PWM 150) 미측정
//
//  [미확정 — 측정 후 반영할 것]
//    ENCODER_CPR    현재 71. 실주행 역산은 약 22.  5m 주행으로 확정 필요
//    전방 오버행    통로폭 계산에 필요
// ----------------------------------------------------------
//
//  조향 부호 : 우 = +, 좌 = -   (팀 규약)
//
//  핀
//    구동 앞  PWM D9  / DIR D10
//    구동 뒤  PWM D7  / DIR D8
//    조향     PWM D11 / DIR D12
//    조향각   A0      (감시 전용. 제어에는 미사용)
//    엔코더   A상 D2  (B상 D3 미사용)
//    E-Stop   D24     (NC 접점. 정상 LOW / 작동·단선 HIGH)
//
//  v30 대비 변경 (v29 기준)
//   조향 엔코더 계측 추가 (D18/D19).
//     A상만 세고 방향은 조향 명령 부호로 부여한다(구동 엔코더와 동일).
//     STATUS 맨 뒤에 필드 2개가 붙는다 — 기존 순서는 그대로다.
//     SZ 명령으로 0 으로 되돌린다.
//     아직 제어에는 쓰지 않는다. 값이 쓸 만한지 먼저 보기 위한 계측용.
//
//  v29 대비 변경 (v28 기준)
//   1. 구동 방향 반전 — 실차에서 전진 명령에 뒤로 굴렀다.
//      앞뒤가 같은 방향으로 함께 뒤로 돌았으므로 전체 부호만 뒤집는다.
//   2. B / BRAKE 명령 추가 — 감속 램프를 건너뛰고 즉시 정지.
//      MD30C 의 DIR 을 토글해 모터 단자를 짧게 단락시키는 다이내믹
//      브레이킹을 시도한다. 브레이크가 없는 차량의 급정거용.
//   3. H<pwm> 명령 추가 — 경사로 홀딩. 램프 없이 지정 PWM 을 즉시 인가해
//      정지 상태를 유지한다. 스톨 전류가 흐르므로 시간 제한을 둔다.
//   4. 부팅 시 BRAKE_MODE / HOLD 관련 값도 출력.
//
//  v27 대비 변경
//    + v25 의 강건 ADC 필터 이식 (중앙값 + 이상치 제거 + 절사평균 + EMA)
//    + 조그 명령이 조향 한계를 넘지 않도록 클램프
//    + 조그 시 steerTargetMs 갱신 누락 수정
//    + 시리얼 오버플로를 FAULT 에서 WARN 으로 완화
//    + parseFloatStrict 에 isfinite 검사 추가
//    + ? 명령으로 명령 목록 출력
// ==========================================================

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ==========================================================
// 1. 핀
// ==========================================================
constexpr uint8_t PWM_DRIVE_FRONT = 9;
constexpr uint8_t DIR_DRIVE_FRONT = 10;
constexpr uint8_t PWM_DRIVE_REAR  = 7;
constexpr uint8_t DIR_DRIVE_REAR  = 8;
constexpr uint8_t PWM_STEER       = 11;
constexpr uint8_t DIR_STEER       = 12;
constexpr uint8_t POT_STEER       = A0;

// ---- v30: 조향 엔코더 ----
//  D2/D3 는 구동 엔코더가 쓰므로 남은 인터럽트 핀을 쓴다.
//  Mega 2560 외부 인터럽트: D2 D3 D18 D19 D20 D21
//  D20/D21 은 I2C 겸용이라 남기고 D18/D19 를 쓴다.
constexpr uint8_t PIN_STEER_ENC_A = 18;
constexpr uint8_t PIN_STEER_ENC_B = 19;   // 현재 미사용 (상태만 읽음)
constexpr unsigned long STEER_ENC_DEBOUNCE_US = 200;
constexpr uint8_t ENC_A           = 2;
constexpr uint8_t ENC_B           = 3;
constexpr uint8_t ESTOP_PIN       = 24;

// ==========================================================
// 2. 모터 방향 극성
//    앞뒤 배선 극성이 서로 반대여서 개별 관리한다.
//    ★ 방향이 뒤집혀 있으면 해당 쌍의 HIGH/LOW 를 교체할 것
// ==========================================================
//  ★ v29: 실차에서 전진 명령에 앞뒤 모두 뒤로 굴러 전체 부호를 뒤집었다.
//     앞뒤 배선 극성이 서로 반대인 것은 그대로 유지된다.
//     (되돌리려면 아래 4줄의 HIGH/LOW 를 서로 바꾸면 된다)
constexpr uint8_t DRIVE_FRONT_FWD = LOW;
constexpr uint8_t DRIVE_FRONT_REV = HIGH;
constexpr uint8_t DRIVE_REAR_FWD  = HIGH;
constexpr uint8_t DRIVE_REAR_REV  = LOW;

// ★ R 명령이 좌로 가면 이 두 줄을 교체할 것
constexpr uint8_t STEER_DIR_LEFT  = HIGH;
constexpr uint8_t STEER_DIR_RIGHT = LOW;

// ==========================================================
// 3. 조향 (시간 기반)
//
//  포텐셔미터가 조향 링키지에 충분히 물려 있지 않아
//  전 구간 A0 변화폭이 실측 약 9카운트뿐이고 노이즈가 그보다 크다.
//  따라서 ADC 폐루프는 성립하지 않으며 A0 는 감시용으로만 쓴다.
//  링키지 개선으로 범위가 200 이상 확보되면 폐루프로 교체할 것.
// ==========================================================
constexpr int     STEER_CENTER_ADC  = 363;   // 직진 정렬 시 실측
constexpr uint8_t STEER_PWM_LEVEL   = 130;
constexpr long    STEER_MIN_MOVE_MS = 30;    // 이보다 작은 이동은 무시
constexpr long    STEER_MAX_MOVE_MS = 1200;  // 1회 최대 이동

long steerNetMs        = 0;                     // 중앙 기준 누적 (우 +, 좌 -)
long steerLimitMs      = 440;                   // 편측 한계.  N 명령
long steerPresetMs[3]  = {150, 300, 440};       // R1/R2/R3.   P 명령

// ==========================================================
// 4. 구동
// ==========================================================
constexpr int DRIVE_PWM_STAGE_1 = 50;
constexpr int DRIVE_PWM_STAGE_2 = 100;
constexpr int DRIVE_PWM_STAGE_3 = 150;

constexpr uint8_t       DRIVE_ACCEL_STEP        = 3;   // 50ms 당 +3  (0→150 약 2.5초)
constexpr uint8_t       DRIVE_DECEL_STEP        = 10;  // 50ms 당 -10 (150→0 약 0.75초)
constexpr unsigned long DRIVE_RAMP_INTERVAL_MS  = 50;
constexpr unsigned long DIRECTION_CHANGE_HOLD_MS = 300;

// ★ 실주행 2000 / 시리얼 수동시험 60000
constexpr unsigned long DRIVE_COMMAND_TIMEOUT_MS = 2000;

// ---- v29: 급정거 (다이내믹 브레이킹) ----
//  MD30C 는 sign-magnitude 방식이라 PWM 을 0 으로 떨구면 코스팅한다.
//  DIR 을 반대로 두고 아주 짧게 PWM 을 인가하면 역토크가 걸려 빨리 선다.
//  BRAKE_PULSE_MS 를 크게 잡으면 역주행이 시작되므로 짧게 유지할 것.
constexpr int           BRAKE_PULSE_PWM = 120;   // 제동 펄스 세기
constexpr unsigned long BRAKE_PULSE_MS  = 150;   // 제동 펄스 길이

// ---- v29: 경사로 홀딩 ----
//  브레이크가 없어 정지 중 중력으로 밀린다. 램프 없이 지정 PWM 을 즉시
//  인가해 버틴다. 스톨 전류가 흐르므로 반드시 시간 제한을 둔다.
constexpr int           HOLD_PWM_MAX    = 90;    // 안전 상한
constexpr unsigned long HOLD_TIMEOUT_MS = 8000;  // 이 시간 지나면 자동 해제

// 워치독 동작
//  true  : 타임아웃 시 FAULT 래치. RESET 해야 재출발
//  false : 감속 정지만. 통신 복구되면 즉시 재개
//  대회 중 일시적 통신 끊김이 완주 실패로 이어지지 않도록 false 를 기본으로 둔다.
constexpr bool WATCHDOG_LATCHES_FAULT = false;

// ==========================================================
// 5. 엔코더
//
//  A/B 쿼드러처는 브러시 노이즈로 방향 부호가 뒤집혀 사용하지 않는다.
//  A상 펄스만 세고 방향은 DIR 상태로 부여한다.
//  디바운스는 실제 펄스 간격보다 충분히 작게 둔다.
//
//  ※ odoCount 는 CPR 과 무관한 원시 펄스 누적이므로
//    5m 캘리브레이션은 CPR 값이 틀려도 그대로 유효하다.
//    CPR 은 RPM 표시에만 영향을 준다.
// ==========================================================
constexpr float         ENCODER_CPR         = 71.0f;  // ★재검증 필요
constexpr uint32_t      ENCODER_DEBOUNCE_US = 200;
constexpr unsigned long ENCODER_UPDATE_MS   = 100;

// ==========================================================
// 6. 조향센서 필터 (v25 방식)
//
//  중앙값 → 이상치 제거 → 절사평균 → EMA → 변화율 제한
//  샘플 수는 v25 의 15 에서 9 로 줄였다.
//  15샘플이면 10ms 주기에서 CPU 의 3분의 1을 점유한다.
// ==========================================================
constexpr uint8_t POT_SAMPLE_COUNT   = 9;
constexpr uint8_t POT_TRIM_COUNT     = 2;
constexpr int     POT_MEDIAN_AROUND  = 60;    // 중앙값에서 이만큼 벗어나면 버림
constexpr float   POT_EMA_ALPHA      = 0.30f;
constexpr int     POT_FILTER_MAX_STEP = 8;    // 1회 출력 변화 상한
constexpr uint16_t POT_SAMPLE_GAP_US = 150;

constexpr int     POT_ELECTRICAL_MIN = 10;
constexpr int     POT_ELECTRICAL_MAX = 1013;
constexpr uint8_t POT_FAULT_CONFIRM_COUNT = 5;

// ==========================================================
// 7. 상태 정의
// ==========================================================
enum class SystemState : uint8_t {
  BOOT,
  READY,
  ACTIVE,
  RESET_REQUIRED,
  ESTOP,
  FAULT
};

enum class FaultCode : uint8_t {
  NONE,
  DRIVE_COMMAND_TIMEOUT,
  STEER_SENSOR_RANGE
};

SystemState systemState = SystemState::BOOT;
FaultCode   faultCode   = FaultCode::NONE;

bool estopActive   = false;
bool resetRequired = false;

// ---- 구동 ----
int  currentDrivePwm = 0;
int  targetDrivePwm  = 0;
bool driveForward    = true;

// ---- v29 ----
bool          brakeActive    = false;   // 제동 펄스 진행 중
unsigned long brakeStartMs   = 0;
bool          holdActive     = false;   // 경사로 홀딩 중
int           holdPwm        = 0;
unsigned long holdStartMs    = 0;

bool directionChangePending = false;
bool pendingDriveForward    = true;
int  pendingDrivePwm        = 0;
bool directionHoldActive    = false;
unsigned long directionHoldStartMs = 0;

unsigned long lastDriveRampMs    = 0;
unsigned long lastDriveCommandMs = 0;

// ---- 조향 (논블로킹) ----
bool          steerRunning   = false;

// ---- v30: 조향 엔코더 ----
volatile long          steerEncCount  = 0;   // 방향 부호 포함 누적
volatile unsigned long steerEncRaw    = 0;   // 부호 없는 원시 엣지 수
volatile unsigned long steerEncLastUs = 0;
volatile int8_t        steerEncDir    = 1;   // 조향 명령 방향 (+우 -좌)
int8_t        steerSign      = 0;
long          steerPlannedMs = 0;
long          steerTargetMs  = 0;
unsigned long steerStartMs   = 0;

// ---- 조향센서 ----
int     currentSteerAdc      = STEER_CENTER_ADC;
uint8_t potRangeFaultCount   = 0;
bool    potWarned            = false;
unsigned long lastPotUpdateMs = 0;

// ---- E-Stop ----
bool          lastEstopReading = false;
unsigned long estopChangeMs    = 0;
constexpr unsigned long ESTOP_DEBOUNCE_MS = 30;

// ---- 엔코더 ----
volatile long     encoderPulse  = 0;
volatile uint32_t encoderLastUs = 0;
long  previousPulse = 0;
long  odoCount      = 0;
float wheelRpm      = 0.0f;
unsigned long lastEncoderUpdateMs = 0;

// ---- 시리얼 ----
constexpr size_t RX_BUFFER_SIZE = 48;
char   rxBuffer[RX_BUFFER_SIZE];
size_t rxIndex = 0;

bool          telemetryEnabled = false;
unsigned long lastTelemetryMs  = 0;

// ==========================================================
// 8. 엔코더 인터럽트
// ==========================================================
void encoderIsr()
{
  uint32_t nowUs = micros();
  if (nowUs - encoderLastUs < ENCODER_DEBOUNCE_US) {
    return;
  }
  encoderLastUs = nowUs;
  encoderPulse++;
}

long readEncoderPulseAtomic()
{
  long value;
  noInterrupts();
  value = encoderPulse;
  interrupts();
  return value;
}

// ==========================================================
// 9. 강건 조향 ADC 필터
// ==========================================================
int insertionSortMedian(int* values, uint8_t count)
{
  for (uint8_t i = 1; i < count; i++) {
    int key = values[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }
  return values[count / 2];
}

int readSteerAdcRobust()
{
  analogRead(POT_STEER);            // 멀티플렉서 안정화용 더미 읽기
  delayMicroseconds(80);

  int samples[POT_SAMPLE_COUNT];
  for (uint8_t i = 0; i < POT_SAMPLE_COUNT; i++) {
    samples[i] = analogRead(POT_STEER);
    delayMicroseconds(POT_SAMPLE_GAP_US);
  }

  int sorted[POT_SAMPLE_COUNT];
  for (uint8_t i = 0; i < POT_SAMPLE_COUNT; i++) {
    sorted[i] = samples[i];
  }
  int median = insertionSortMedian(sorted, POT_SAMPLE_COUNT);

  // 중앙값에서 크게 벗어난 표본을 버린다.
  int kept[POT_SAMPLE_COUNT];
  uint8_t keptCount = 0;
  for (uint8_t i = 0; i < POT_SAMPLE_COUNT; i++) {
    if (abs(samples[i] - median) <= POT_MEDIAN_AROUND) {
      kept[keptCount++] = samples[i];
    }
  }

  float filtered = (float)median;

  // 남은 표본이 충분하면 양끝을 잘라내고 평균한다.
  if (keptCount >= (2 * POT_TRIM_COUNT + 1)) {
    insertionSortMedian(kept, keptCount);      // 정렬 목적
    long  sum   = 0;
    uint8_t cnt = 0;
    for (uint8_t i = POT_TRIM_COUNT; i < keptCount - POT_TRIM_COUNT; i++) {
      sum += kept[i];
      cnt++;
    }
    if (cnt > 0) {
      filtered = (float)sum / (float)cnt;
    }
  }

  static bool  emaInit = false;
  static float ema     = 0.0f;
  static int   lastOut = STEER_CENTER_ADC;

  if (!emaInit) {
    ema     = filtered;
    emaInit = true;
  }
  else {
    ema = POT_EMA_ALPHA * filtered + (1.0f - POT_EMA_ALPHA) * ema;
  }

  int candidate = (int)lroundf(ema);

  // 급변 제한
  int diff = candidate - lastOut;
  if (diff > POT_FILTER_MAX_STEP) {
    candidate = lastOut + POT_FILTER_MAX_STEP;
  }
  else if (diff < -POT_FILTER_MAX_STEP) {
    candidate = lastOut - POT_FILTER_MAX_STEP;
  }

  lastOut = candidate;
  return constrain(candidate, 0, 1023);
}

// ==========================================================
// 10. 모터 저수준 출력
// ==========================================================
void applyDrivePwm(int pwm)
{
  currentDrivePwm = constrain(pwm, 0, 255);
  analogWrite(PWM_DRIVE_FRONT, currentDrivePwm);
  analogWrite(PWM_DRIVE_REAR,  currentDrivePwm);
}

void setDriveDirection(bool forward)
{
  driveForward = forward;
  digitalWrite(DIR_DRIVE_FRONT, forward ? DRIVE_FRONT_FWD : DRIVE_FRONT_REV);
  digitalWrite(DIR_DRIVE_REAR,  forward ? DRIVE_REAR_FWD  : DRIVE_REAR_REV);
}

void stopSteerMotor()
{
  analogWrite(PWM_STEER, 0);
}

// ==========================================================
// 11. 상태·고장 이름
// ==========================================================
const __FlashStringHelper* stateName()
{
  switch (systemState) {
    case SystemState::BOOT:           return F("BOOT");
    case SystemState::READY:          return F("READY");
    case SystemState::ACTIVE:         return F("ACTIVE");
    case SystemState::RESET_REQUIRED: return F("RESET_REQUIRED");
    case SystemState::ESTOP:          return F("ESTOP");
    case SystemState::FAULT:          return F("FAULT");
  }
  return F("UNKNOWN");
}

const __FlashStringHelper* faultName()
{
  switch (faultCode) {
    case FaultCode::NONE:                  return F("NONE");
    case FaultCode::DRIVE_COMMAND_TIMEOUT: return F("DRIVE_COMMAND_TIMEOUT");
    case FaultCode::STEER_SENSOR_RANGE:    return F("STEER_SENSOR_RANGE");
  }
  return F("UNKNOWN");
}

// ==========================================================
// 12. 조향 제어 (논블로킹)
//
//  블로킹 delay() 를 쓰면 조향하는 동안 워치독·E-Stop·시리얼이
//  전부 멈춘다. 시작만 하고 loop 에서 완료를 감시한다.
// ==========================================================
void cancelSteer()
{
  if (!steerRunning) {
    return;
  }
  // 실제로 움직인 만큼만 누적에 반영한다.
  long elapsed = (long)(millis() - steerStartMs);
  if (elapsed > steerPlannedMs) {
    elapsed = steerPlannedMs;
  }
  steerNetMs += (long)steerSign * elapsed;
  stopSteerMotor();
  steerRunning   = false;
  steerSign      = 0;
  steerPlannedMs = 0;
}

// sign > 0 = 우, sign < 0 = 좌
// 조향 한계를 넘지 않도록 이동량을 잘라낸다.
// ==========================================================
// v30. 조향 엔코더 ISR
//
//  A상만 센다. 방향은 조향 명령 부호로 부여한다.
//  A/B 쿼드러처는 브러시 노이즈에 취약해 구동 엔코더에서 이미 버린 방식이다.
// ==========================================================
void steerEncIsr()
{
  unsigned long now = micros();
  if (now - steerEncLastUs < STEER_ENC_DEBOUNCE_US) {
    return;
  }
  steerEncLastUs = now;
  steerEncRaw++;
  steerEncCount += (steerEncDir >= 0) ? 1 : -1;
}

void startSteer(int8_t sign, long ms)
{
  cancelSteer();

  if (ms <= 0) {
    return;
  }
  if (ms > STEER_MAX_MOVE_MS) {
    ms = STEER_MAX_MOVE_MS;
  }

  // 한계까지 남은 여유
  long room = (sign > 0) ? (steerLimitMs - steerNetMs)
                         : (steerNetMs + steerLimitMs);
  if (room <= 0) {
    Serial.print(F("STEER_LIMIT_REACHED,"));
    Serial.println(steerNetMs);
    return;
  }
  if (ms > room) {
    ms = room;
  }

  digitalWrite(DIR_STEER, sign > 0 ? STEER_DIR_RIGHT : STEER_DIR_LEFT);
  steerEncDir = sign;               // v30: 엔코더 부호용
  analogWrite(PWM_STEER, STEER_PWM_LEVEL);

  steerRunning   = true;
  steerSign      = sign;
  steerPlannedMs = ms;
  steerStartMs   = millis();
  steerTargetMs  = steerNetMs + (long)sign * ms;
}

// 중앙 기준 목표 누적 ms 로 이동
void gotoSteerMs(long target)
{
  target = constrain(target, -steerLimitMs, steerLimitMs);

  long diff = target - steerNetMs;
  if (labs(diff) < STEER_MIN_MOVE_MS) {
    steerTargetMs = steerNetMs;
    Serial.println(F("STEER_OK,ALREADY"));
    return;
  }
  startSteer(diff > 0 ? 1 : -1, labs(diff));
  steerTargetMs = target;
}

void updateSteering(unsigned long now)
{
  if (!steerRunning) {
    return;
  }
  if ((long)(now - steerStartMs) < steerPlannedMs) {
    return;
  }

  stopSteerMotor();
  steerNetMs += (long)steerSign * steerPlannedMs;
  steerRunning   = false;
  steerSign      = 0;
  steerPlannedMs = 0;

  Serial.print(F("STEER_OK,"));
  Serial.print(steerNetMs);
  Serial.print(',');
  Serial.println(currentSteerAdc);
}

// ==========================================================
// 13. 정지 및 FAULT
// ==========================================================
void forceImmediateStop()
{
  targetDrivePwm         = 0;
  pendingDrivePwm        = 0;
  directionChangePending = false;
  directionHoldActive    = false;
  brakeActive            = false;
  holdActive             = false;
  applyDrivePwm(0);
  cancelSteer();
}

// ==========================================================
// 13-b. v29 급정거 — 다이내믹 브레이킹
//
//  감속 램프(50ms 당 -10)를 건너뛰고 즉시 세운다.
//  DIR 을 반대로 두고 짧은 역토크 펄스를 준 뒤 PWM 0 으로 떨어뜨린다.
//  펄스가 길면 역주행이 시작되므로 BRAKE_PULSE_MS 를 반드시 짧게 둘 것.
//
//  주의: 모터·기어·드라이버에 부담이 간다. 비상시에만 쓴다.
// ==========================================================
void requestBrake()
{
  lastDriveCommandMs = millis();

  // 이미 서 있으면 펄스 없이 끝낸다 (불필요한 역토크 방지)
  if (currentDrivePwm == 0 && !brakeActive) {
    targetDrivePwm         = 0;
    pendingDrivePwm        = 0;
    directionChangePending = false;
    directionHoldActive    = false;
    holdActive             = false;
    applyDrivePwm(0);
    Serial.println(F("BRAKE_OK,ALREADY_STOPPED"));
    return;
  }

  // 예약된 방향 전환·홀딩을 모두 취소
  targetDrivePwm         = 0;
  pendingDrivePwm        = 0;
  directionChangePending = false;
  directionHoldActive    = false;
  holdActive             = false;

  // 역토크 펄스: 현재 진행 방향의 반대로 DIR 을 두고 짧게 인가
  applyDrivePwm(0);
  digitalWrite(DIR_DRIVE_FRONT, driveForward ? DRIVE_FRONT_REV : DRIVE_FRONT_FWD);
  digitalWrite(DIR_DRIVE_REAR,  driveForward ? DRIVE_REAR_REV  : DRIVE_REAR_FWD);
  applyDrivePwm(BRAKE_PULSE_PWM);

  brakeActive  = true;
  brakeStartMs = millis();

  Serial.print(F("BRAKE_OK,"));
  Serial.println(BRAKE_PULSE_MS);
}

void updateBrake(unsigned long now)
{
  if (!brakeActive) {
    return;
  }
  if (now - brakeStartMs < BRAKE_PULSE_MS) {
    return;
  }
  // 펄스 종료 → 완전 정지. DIR 은 원래 진행 방향으로 되돌린다.
  applyDrivePwm(0);
  targetDrivePwm = 0;
  setDriveDirection(driveForward);
  brakeActive = false;
  Serial.println(F("BRAKE_DONE"));
}

// ==========================================================
// 13-c. v29 경사로 홀딩
//
//  브레이크가 없어 경사로에서 정지하면 중력으로 밀린다.
//  램프를 거치지 않고 지정 PWM 을 즉시 인가해 버틴다.
//  스톨 상태라 전류가 크게 흐르므로 HOLD_TIMEOUT_MS 로 자동 해제한다.
//
//  사용: H40  →  PWM 40 으로 전진 방향 홀딩
//        H0 또는 다른 구동 명령으로 해제
// ==========================================================
void requestHold(int pwm)
{
  lastDriveCommandMs = millis();

  if (pwm <= 0) {
    holdActive = false;
    applyDrivePwm(0);
    targetDrivePwm = 0;
    Serial.println(F("HOLD_OFF"));
    return;
  }

  pwm = constrain(pwm, 0, HOLD_PWM_MAX);

  // 홀딩은 정지 상태에서만 시작한다. 달리는 중이면 거부.
  if (currentDrivePwm > 0 && !holdActive) {
    Serial.println(F("HOLD_REJECTED,MOVING"));
    return;
  }

  directionChangePending = false;
  directionHoldActive    = false;
  brakeActive            = false;

  setDriveDirection(true);          // 오르막에서 밀림을 막으려면 전진 토크
  applyDrivePwm(pwm);
  targetDrivePwm = pwm;

  holdActive  = true;
  holdPwm     = pwm;
  holdStartMs = millis();

  Serial.print(F("HOLD_OK,"));
  Serial.println(pwm);
}

void updateHold(unsigned long now)
{
  if (!holdActive) {
    return;
  }
  if (now - holdStartMs < HOLD_TIMEOUT_MS) {
    // 램프가 홀딩 PWM 을 갉아먹지 않도록 목표를 계속 고정한다.
    targetDrivePwm = holdPwm;
    return;
  }
  // 과열 방지를 위한 자동 해제
  holdActive     = false;
  targetDrivePwm = 0;
  applyDrivePwm(0);
  Serial.println(F("HOLD_TIMEOUT"));
}

void latchFault(FaultCode code)
{
  if (systemState == SystemState::FAULT) {
    return;
  }
  faultCode = code;
  forceImmediateStop();
  resetRequired = true;
  systemState   = SystemState::FAULT;

  Serial.print(F("FAULT,"));
  Serial.println(faultName());
}

// ==========================================================
// 14. E-Stop  (NC 접점. 단선도 정지로 처리된다)
// ==========================================================
void updateEstop(unsigned long now)
{
  bool reading = (digitalRead(ESTOP_PIN) == HIGH);

  if (reading != lastEstopReading) {
    estopChangeMs    = now;
    lastEstopReading = reading;
    return;
  }
  if (now - estopChangeMs < ESTOP_DEBOUNCE_MS) {
    return;
  }
  if (reading == estopActive) {
    return;
  }

  estopActive = reading;

  if (estopActive) {
    forceImmediateStop();
    resetRequired = true;
    systemState   = SystemState::ESTOP;
    Serial.println(F("FAULT,ESTOP"));
  }
  else {
    resetRequired = true;
    systemState   = SystemState::RESET_REQUIRED;
    Serial.println(F("STATE,RESET_REQUIRED"));
  }
}

// ==========================================================
// 15. 조향센서 감시
//
//  포텐셔미터는 현재 제어에 쓰이지 않으므로
//  이상이 있어도 차량을 정지시키지 않고 경고만 낸다.
//  링키지 개선으로 폐루프를 도입하면 FAULT 로 승격할 것.
// ==========================================================
void updateSteerSensor(unsigned long now)
{
  if (now - lastPotUpdateMs < 20) {
    return;
  }
  lastPotUpdateMs = now;
  currentSteerAdc = readSteerAdcRobust();

  bool valid = (currentSteerAdc >= POT_ELECTRICAL_MIN &&
                currentSteerAdc <= POT_ELECTRICAL_MAX);

  if (valid) {
    potRangeFaultCount = 0;
    potWarned = false;
    return;
  }

  if (potRangeFaultCount < 255) {
    potRangeFaultCount++;
  }
  if (potRangeFaultCount >= POT_FAULT_CONFIRM_COUNT && !potWarned) {
    potWarned = true;
    Serial.print(F("WARN,STEER_SENSOR_RANGE,"));
    Serial.println(currentSteerAdc);
  }
}

// ==========================================================
// 16. 구동 제어
// ==========================================================
int stageToPwm(float stage, bool &forward)
{
  forward = true;
  if (fabs(stage - 1.0f) < 0.01f) return 0;
  if (fabs(stage - 2.0f) < 0.01f) return DRIVE_PWM_STAGE_1;
  if (fabs(stage - 3.0f) < 0.01f) return DRIVE_PWM_STAGE_2;
  if (fabs(stage - 4.0f) < 0.01f) return DRIVE_PWM_STAGE_3;

  forward = false;
  if (fabs(stage - 5.0f) < 0.01f) return 0;
  if (fabs(stage - 6.0f) < 0.01f) return DRIVE_PWM_STAGE_1;
  if (fabs(stage - 7.0f) < 0.01f) return DRIVE_PWM_STAGE_2;
  if (fabs(stage - 8.0f) < 0.01f) return DRIVE_PWM_STAGE_3;

  return -1;
}

void requestDrive(bool forward, int pwm)
{
  lastDriveCommandMs = millis();

  // v29: 일반 구동 명령이 오면 제동·홀딩 상태를 먼저 해제한다.
  if (brakeActive) {
    brakeActive = false;
    applyDrivePwm(0);
    setDriveDirection(driveForward);
  }
  holdActive = false;

  if (pwm == 0) {
    targetDrivePwm         = 0;
    directionChangePending = false;
    directionHoldActive    = false;
    return;
  }

  // 정지 상태 → 즉시 방향 설정
  if (currentDrivePwm == 0 && targetDrivePwm == 0 && !directionChangePending) {
    setDriveDirection(forward);
    targetDrivePwm = pwm;
    return;
  }

  // 같은 방향 → 속도만 변경
  if (forward == driveForward && !directionChangePending) {
    targetDrivePwm = pwm;
    return;
  }

  // 반대 방향 → 감속 후 전환 예약
  directionChangePending = true;
  pendingDriveForward    = forward;
  pendingDrivePwm        = pwm;
  targetDrivePwm         = 0;
  directionHoldActive    = false;

  Serial.println(F("DRIVE,DIRECTION_CHANGE_PENDING"));
}

void updateDriveController(unsigned long now)
{
  // v29: 제동 펄스 중에는 램프가 PWM 을 건드리면 안 된다.
  if (brakeActive) {
    return;
  }
  if (now - lastDriveRampMs < DRIVE_RAMP_INTERVAL_MS) {
    return;
  }
  lastDriveRampMs = now;

  if (currentDrivePwm < targetDrivePwm) {
    applyDrivePwm(min(currentDrivePwm + DRIVE_ACCEL_STEP, targetDrivePwm));
  }
  else if (currentDrivePwm > targetDrivePwm) {
    applyDrivePwm(max(currentDrivePwm - DRIVE_DECEL_STEP, targetDrivePwm));
  }

  if (!directionChangePending || currentDrivePwm != 0) {
    return;
  }

  // 완전 정지 후 일정 시간 유지한 뒤에만 방향을 바꾼다.
  // 회전 중 DIR 을 뒤집으면 역기전력이 드라이버를 손상시킨다.
  if (!directionHoldActive) {
    directionHoldActive  = true;
    directionHoldStartMs = now;
    return;
  }
  if (now - directionHoldStartMs < DIRECTION_CHANGE_HOLD_MS) {
    return;
  }

  setDriveDirection(pendingDriveForward);
  targetDrivePwm         = pendingDrivePwm;
  directionChangePending = false;
  directionHoldActive    = false;

  Serial.println(F("DRIVE,DIRECTION_CHANGE_OK"));
}

void updateDriveWatchdog(unsigned long now)
{
  bool active = (currentDrivePwm > 0 ||
                 targetDrivePwm > 0 ||
                 directionChangePending);
  if (!active) {
    return;
  }
  if (now - lastDriveCommandMs <= DRIVE_COMMAND_TIMEOUT_MS) {
    return;
  }

  if (WATCHDOG_LATCHES_FAULT) {
    latchFault(FaultCode::DRIVE_COMMAND_TIMEOUT);
  }
  else {
    targetDrivePwm         = 0;
    directionChangePending = false;
    directionHoldActive    = false;
    Serial.println(F("DRIVE,WATCHDOG_STOP"));
  }
}

// ==========================================================
// 17. 엔코더 갱신
// ==========================================================
void updateEncoder(unsigned long now)
{
  unsigned long elapsed = now - lastEncoderUpdateMs;
  if (elapsed < ENCODER_UPDATE_MS) {
    return;
  }

  long pulse = readEncoderPulseAtomic();
  long delta = pulse - previousPulse;
  previousPulse = pulse;

  odoCount += driveForward ? delta : -delta;

  float rawRpm = ((float)delta / ENCODER_CPR) * (60000.0f / (float)elapsed);
  if (!driveForward) {
    rawRpm = -rawRpm;
  }

  static float   rpmBuffer[5] = {0, 0, 0, 0, 0};
  static uint8_t rpmIndex = 0;
  rpmBuffer[rpmIndex] = rawRpm;
  rpmIndex = (rpmIndex + 1) % 5;

  float sum = 0.0f;
  for (uint8_t i = 0; i < 5; i++) {
    sum += rpmBuffer[i];
  }
  wheelRpm = sum / 5.0f;

  lastEncoderUpdateMs = now;
}

// ==========================================================
// 18. 파싱 보조
// ==========================================================
bool parseFloatStrict(const char* text, float &value)
{
  char* endPointer = NULL;
  value = strtod(text, &endPointer);
  if (endPointer == text) {
    return false;
  }
  while (*endPointer == ' ') {
    endPointer++;
  }
  return (*endPointer == '\0') && isfinite(value);
}

bool parseLongStrict(const char* text, long &value)
{
  char* endPointer = NULL;
  value = strtol(text, &endPointer, 10);
  if (endPointer == text) {
    return false;
  }
  while (*endPointer == ' ') {
    endPointer++;
  }
  return (*endPointer == '\0');
}

void uppercase(char* text)
{
  while (*text != '\0') {
    if (*text >= 'a' && *text <= 'z') {
      *text = (char)(*text - 'a' + 'A');
    }
    text++;
  }
}

// ==========================================================
// 19. 상태 출력
//
//  기존 브릿지 파서 호환을 위해 앞 7필드 순서를 고정한다.
//  STATUS,state,fault,adc,target_adc,drive_pwm,rpm,encoder_count
//         ,steer_net_ms,steer_target_ms,steer_limit_ms
//         ,steer_enc,steer_enc_raw        <- v30 에서 맨 뒤에 추가
//                                    ↑ ADC 폐루프 미사용이라 고정값
//  이후 3필드는 v27 에서 뒤에 추가한 것이다.
//  추가는 항상 맨 뒤에만 하므로 기존 파서는 깨지지 않는다.
// ==========================================================
void publishStatus()
{
  Serial.print(F("STATUS,"));
  Serial.print(stateName());
  Serial.print(',');
  Serial.print(faultName());
  Serial.print(',');
  Serial.print(currentSteerAdc);
  Serial.print(',');
  Serial.print(STEER_CENTER_ADC);
  Serial.print(',');
  Serial.print(currentDrivePwm);
  Serial.print(',');
  Serial.print(wheelRpm, 2);
  Serial.print(',');
  Serial.print(odoCount);
  Serial.print(',');
  Serial.print(steerNetMs);
  Serial.print(',');
  Serial.print(steerTargetMs);
  Serial.print(',');
  Serial.print(steerLimitMs);

  // v30: 조향 엔코더 (맨 뒤에 추가. 기존 필드 순서는 그대로다)
  noInterrupts();
  long sEnc = steerEncCount;
  unsigned long sRaw = steerEncRaw;
  interrupts();
  Serial.print(',');
  Serial.print(sEnc);
  Serial.print(',');
  Serial.println(sRaw);
}

void publishHelp()
{
  Serial.println(F("-- DRIVE --"));
  Serial.println(F("1.00 stop / 2~4 fwd / 6~8 rev"));
  Serial.println(F("-- STEER (R=+ right, L=- left) --"));
  Serial.println(F("R1 R2 R3 / L1 L2 L3 / C center"));
  Serial.println(F("V,0.5 normalized / W300 absolute ms"));
  Serial.println(F("A200 jog left / D200 jog right"));
  Serial.println(F("M set zero"));
  Serial.println(F("-- CONFIG --"));
  Serial.println(F("P150,300,440 preset / N440 limit"));
  Serial.println(F("-- SYSTEM --"));
  Serial.println(F("S status / Q telemetry / O odo reset"));
  Serial.println(F("X stop / RESET / ? help"));
  Serial.println(F("-- v29 --"));
  Serial.println(F("B brake (skip ramp, emergency)"));
  Serial.println(F("H40 slope hold pwm / H0 release"));
  Serial.println(F("-- v30 --"));
  Serial.println(F("SZ steering encoder reset (D18)"));
}

// ==========================================================
// 20. RESET
// ==========================================================
void resetSystem()
{
  if (estopActive) {
    Serial.println(F("RESET_REJECTED,ESTOP"));
    return;
  }

  forceImmediateStop();
  faultCode          = FaultCode::NONE;
  resetRequired      = false;
  potRangeFaultCount = 0;
  potWarned          = false;
  lastDriveCommandMs = millis();
  systemState        = SystemState::READY;

  Serial.println(F("RESET_OK"));
  Serial.println(F("STATE,READY"));
}

// ==========================================================
// 21. 명령 처리
// ==========================================================
void processCommand(char* command)
{
  while (*command == ' ') {
    command++;
  }
  size_t length = strlen(command);
  while (length > 0 && command[length - 1] == ' ') {
    command[length - 1] = '\0';
    length--;
  }
  if (length == 0) {
    return;
  }

  uppercase(command);

  // ---- 상태와 무관하게 항상 받는 명령 ----
  if (strcmp(command, "RESET") == 0) { resetSystem();  return; }
  if (strcmp(command, "?")     == 0) { publishHelp();  return; }
  if (strcmp(command, "S") == 0 || strcmp(command, "STATUS") == 0) {
    publishStatus();
    return;
  }
  if (strcmp(command, "Q") == 0) {
    telemetryEnabled = !telemetryEnabled;
    Serial.print(F("TELEMETRY,"));
    Serial.println(telemetryEnabled ? F("ON") : F("OFF"));
    return;
  }
  if (strcmp(command, "X") == 0 || strcmp(command, "STOP") == 0) {
    forceImmediateStop();
    if (!estopActive && faultCode == FaultCode::NONE) {
      systemState = SystemState::READY;
    }
    Serial.println(F("STOP_OK"));
    return;
  }
  // v29: 급정거. 감속 램프를 건너뛴다. 상태와 무관하게 항상 받는다.
  if (strcmp(command, "B") == 0 || strcmp(command, "BRAKE") == 0) {
    requestBrake();
    return;
  }

  // ---- 이후는 READY / ACTIVE 에서만 ----
  if (systemState != SystemState::READY &&
      systemState != SystemState::ACTIVE) {
    Serial.println(F("COMMAND_REJECTED,NOT_READY"));
    return;
  }

  // v29: 경사로 홀딩.  H40 → PWM 40 으로 버팀,  H0 → 해제
  if (command[0] == 'H' && length >= 2) {
    long value;
    if (!parseLongStrict(command + 1, value)) {
      Serial.println(F("HOLD_ERR,BAD_VALUE"));
      return;
    }
    requestHold((int)value);
    return;
  }

  // v30: 조향 엔코더 리셋
  if (strcmp(command, "SZ") == 0) {
    noInterrupts();
    steerEncCount = 0;
    steerEncRaw   = 0;
    interrupts();
    Serial.println(F("STEER_ENC_RESET"));
    return;
  }

  // 오도미터 리셋 (5m 캘리브레이션용)
  if (strcmp(command, "O") == 0) {
    noInterrupts();
    encoderPulse = 0;
    interrupts();
    previousPulse = 0;
    odoCount      = 0;
    Serial.println(F("ODO_RESET"));
    return;
  }

  // 조향 누적 영점 — 앞바퀴를 직진 정렬한 뒤 실행할 것
  if (strcmp(command, "M") == 0) {
    cancelSteer();
    steerNetMs    = 0;
    steerTargetMs = 0;
    Serial.print(F("STEER_ZERO,"));
    Serial.println(currentSteerAdc);
    return;
  }

  // 중앙 복귀
  if (strcmp(command, "C") == 0 || strcmp(command, "T") == 0) {
    gotoSteerMs(0);
    return;
  }

  // 조그 — A 좌 / D 우
  if (command[0] == 'A' || command[0] == 'D') {
    long duration = 200;
    if (command[1] != '\0' && !parseLongStrict(command + 1, duration)) {
      Serial.print(F("COMMAND_ERROR,"));
      Serial.println(command[0]);
      return;
    }
    startSteer(command[0] == 'D' ? 1 : -1, duration);
    return;
  }

  // 프리셋 변경 : P150,300,440
  if (command[0] == 'P') {
    char* first = strchr(command + 1, ',');
    if (first == NULL) {
      Serial.println(F("COMMAND_ERROR,P"));
      return;
    }
    char* second = strchr(first + 1, ',');
    if (second == NULL) {
      Serial.println(F("COMMAND_ERROR,P"));
      return;
    }
    *first  = '\0';
    *second = '\0';

    long a, b, c;
    if (!parseLongStrict(command + 1, a) ||
        !parseLongStrict(first + 1, b) ||
        !parseLongStrict(second + 1, c)) {
      Serial.println(F("COMMAND_ERROR,P"));
      return;
    }
    steerPresetMs[0] = a;
    steerPresetMs[1] = b;
    steerPresetMs[2] = c;

    Serial.print(F("PRESET_OK,"));
    Serial.print(a); Serial.print(',');
    Serial.print(b); Serial.print(',');
    Serial.println(c);

    if (c > steerLimitMs) {
      Serial.println(F("WARN,PRESET3_OVER_LIMIT"));
    }
    return;
  }

  // 조향 한계 : N440
  if (command[0] == 'N') {
    long value;
    if (!parseLongStrict(command + 1, value)) {
      Serial.println(F("COMMAND_ERROR,N"));
      return;
    }
    if (value < 100 || value > 2000) {
      Serial.println(F("COMMAND_ERROR,N_RANGE"));
      return;
    }
    steerLimitMs = value;
    Serial.print(F("LIMIT_OK,"));
    Serial.println(steerLimitMs);
    return;
  }

  // 절대 조향 : W300 / W-300   (/mcu_wheel 스케일 그대로)
  if (command[0] == 'W') {
    long value;
    if (!parseLongStrict(command + 1, value)) {
      Serial.println(F("COMMAND_ERROR,W"));
      return;
    }
    gotoSteerMs(value);
    return;
  }

  // 단계 조향 : R1~R3 (우, +) / L1~L3 (좌, -)
  if ((command[0] == 'L' || command[0] == 'R') &&
      command[1] >= '1' && command[1] <= '3' &&
      command[2] == '\0') {
    long magnitude = steerPresetMs[command[1] - '1'];
    gotoSteerMs(command[0] == 'R' ? magnitude : -magnitude);
    return;
  }

  // 정규화 조향 : V,0.5  (양수 우 / 음수 좌)
  if (command[0] == 'V' && command[1] == ',') {
    float normalized;
    if (!parseFloatStrict(command + 2, normalized)) {
      Serial.println(F("COMMAND_ERROR,V"));
      return;
    }
    normalized = constrain(normalized, -1.0f, 1.0f);
    gotoSteerMs((long)(normalized * (float)steerLimitMs));
    return;
  }

  // 숫자 = 구동 단계
  float stage;
  if (!parseFloatStrict(command, stage)) {
    Serial.println(F("COMMAND_ERROR,UNKNOWN"));
    return;
  }

  bool forward;
  int pwm = stageToPwm(stage, forward);
  if (pwm < 0) {
    forceImmediateStop();
    Serial.println(F("COMMAND_ERROR,DRIVE_STAGE"));
    return;
  }

  requestDrive(forward, pwm);
}

// ==========================================================
// 22. 시리얼 수신
//
//  버퍼가 넘치면 해당 줄만 버린다.
//  잡음 한 번으로 차량 전체를 정지시키지 않기 위해 FAULT 로 올리지 않는다.
// ==========================================================
void updateSerialInput()
{
  static bool overflowed = false;

  while (Serial.available() > 0) {
    char received = (char)Serial.read();

    if (received == '\n') {
      if (overflowed) {
        overflowed = false;
        rxIndex = 0;
        Serial.println(F("WARN,SERIAL_OVERFLOW"));
        continue;
      }
      rxBuffer[rxIndex] = '\0';
      processCommand(rxBuffer);
      rxIndex = 0;
    }
    else if (received != '\r') {
      if (rxIndex < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxIndex++] = received;
      }
      else {
        overflowed = true;     // 개행이 올 때까지 나머지를 버린다
      }
    }
  }
}

// ==========================================================
// 23. 상태 갱신 및 텔레메트리
// ==========================================================
void updateSystemState()
{
  if (systemState != SystemState::READY &&
      systemState != SystemState::ACTIVE) {
    return;
  }
  bool active = (currentDrivePwm > 0 ||
                 targetDrivePwm > 0 ||
                 directionChangePending ||
                 steerRunning);
  systemState = active ? SystemState::ACTIVE : SystemState::READY;
}

void updateTelemetry(unsigned long now)
{
  if (!telemetryEnabled) {
    return;
  }
  if (now - lastTelemetryMs < 500) {
    return;
  }
  lastTelemetryMs = now;
  publishStatus();
}

// ==========================================================
// 24. setup
// ==========================================================
void setup()
{
  Serial.begin(115200);

  pinMode(PWM_DRIVE_FRONT, OUTPUT);
  pinMode(DIR_DRIVE_FRONT, OUTPUT);
  pinMode(PWM_DRIVE_REAR,  OUTPUT);
  pinMode(DIR_DRIVE_REAR,  OUTPUT);
  pinMode(PWM_STEER,       OUTPUT);
  pinMode(DIR_STEER,       OUTPUT);
  pinMode(POT_STEER,       INPUT);
  pinMode(ENC_A,     INPUT_PULLUP);
  pinMode(ENC_B,     INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  setDriveDirection(true);
  applyDrivePwm(0);
  stopSteerMotor();

  encoderLastUs = micros();
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderIsr, RISING);

  // v30: 조향 엔코더. 내부 풀업을 켜 오픈컬렉터/접점식도 읽는다.
  pinMode(PIN_STEER_ENC_A, INPUT_PULLUP);
  pinMode(PIN_STEER_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_STEER_ENC_A), steerEncIsr, CHANGE);

  delay(300);
  currentSteerAdc = readSteerAdcRobust();
  steerNetMs      = 0;
  steerTargetMs   = 0;

  unsigned long now = millis();
  lastDriveRampMs     = now;
  lastDriveCommandMs  = now;
  lastEncoderUpdateMs = now;
  lastPotUpdateMs     = now;
  estopChangeMs       = now;

  lastEstopReading = (digitalRead(ESTOP_PIN) == HIGH);
  estopActive      = lastEstopReading;

  Serial.println(F("MCU_BOOT,v30"));
  Serial.print(F("CENTER_ADC,"));   Serial.println(STEER_CENTER_ADC);
  Serial.print(F("CURRENT_ADC,"));  Serial.println(currentSteerAdc);
  Serial.print(F("STEER_LIMIT,"));  Serial.println(steerLimitMs);
  Serial.print(F("WATCHDOG_MS,"));  Serial.println(DRIVE_COMMAND_TIMEOUT_MS);
  Serial.print(F("BRAKE_PULSE,"));  Serial.print(BRAKE_PULSE_PWM);
  Serial.print(',');                Serial.println(BRAKE_PULSE_MS);
  Serial.print(F("HOLD_PWM_MAX,")); Serial.println(HOLD_PWM_MAX);
  Serial.print(F("STEER_ENC_PIN,")); Serial.print(PIN_STEER_ENC_A);
  Serial.print(',');                 Serial.println(PIN_STEER_ENC_B);

  if (estopActive) {
    resetRequired = true;
    systemState   = SystemState::ESTOP;
    Serial.println(F("FAULT,ESTOP"));
  }
  else {
    // 부팅 시 조향 모터를 움직이지 않는다.
    // 현재 조향 위치를 그대로 기준으로 삼으므로,
    // 앞바퀴를 직진 정렬한 뒤 M 으로 영점을 다시 잡을 것.
    systemState = SystemState::READY;
    Serial.println(F("STATE,READY"));
  }
}

// ==========================================================
// 25. loop
// ==========================================================
void loop()
{
  unsigned long now = millis();

  // 안전 입력을 가장 먼저 본다.
  updateEstop(now);

  updateSerialInput();
  updateSteerSensor(now);

  if (systemState == SystemState::ESTOP ||
      systemState == SystemState::FAULT ||
      systemState == SystemState::RESET_REQUIRED) {
    applyDrivePwm(0);
    stopSteerMotor();
    targetDrivePwm         = 0;
    directionChangePending = false;
    steerRunning           = false;
    brakeActive            = false;   // v29
    holdActive             = false;   // v29
  }
  else {
    updateBrake(now);                 // v29: 램프보다 먼저
    updateHold(now);                  // v29
    updateDriveWatchdog(now);
    updateDriveController(now);
    updateSteering(now);
  }

  updateEncoder(now);
  updateSystemState();
  updateTelemetry(now);
}
