// ==========================================================
//  T870 하위제어기 v35
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
//    3.00 (PWM 100) 0.526 m/s  (0829 하중 실린 상태 실측 0.455)
//    4.00 (PWM 150) 미측정
//    counts_per_meter  199.8   (0829 실측. 1카운트 5.01mm)
//
//  [미확정 — 측정 후 반영할 것]
//    전방 오버행    통로폭 계산에 필요
//    코스팅 거리    브레이크가 없어 정지 명령 후 굴러가는 거리
//    지면 조향각    27도는 무부하 실측값이다
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
//  v35 대비 변경 (v34 기준)  — 2026-08-29
//   ★ 경사로 자동 안티롤백 (AR)
//
//   1. 정지 명령 상태에서 엔코더가 저 혼자 올라가면 = 차가 굴러가고 있다.
//      버티는 방향으로 짧은 토크 펄스를 넣어 제자리를 지킨다.
//      경사로에 세워두면 매번 뒤로 밀리던 문제를 없앤다.
//
//   2. 우리 엔코더는 A상 단상이라 부호가 없다.
//      "정지 명령인데 카운트가 는다" 를 밀림으로 판정한다.
//      그래서 어느 쪽으로 미는지는 사람이 알려줘야 한다.
//        AR1 = 전진 토크로 버팀 (오르막에 앞으로 세워둔 보통 상황)
//        AR2 = 후진 토크로 버팀 (내리막)
//        AR0 = 끔
//      ⚠ 방향을 반대로 켜면 차를 스스로 밀어낸다. 반드시 확인할 것.
//      그 사고를 막으려고 폭주 감지(ANTIROLL_ABORT_ENGAGES)를 넣었다.
//
//   3. 토크 펄스는 제동(v33)과 같이 엔코더가 끊는다.
//      잘 잡히면 100ms 언저리에서 끝나고, 잘 안 잡히면 1.2초까지
//      늘어난다 = 사실상 연속 홀딩. 필요한 만큼만 문다.
//
//   4. 안전 장치 5중
//      - 1회 연속 유지 절대 상한  ANTIROLL_HOLD_MAX_MS (1.2초)
//      - 쿨다운을 직전 유지시간 이상으로 → 스톨 듀티 50% 를 못 넘는다
//      - 폭주 감지: 토크 중 1.0m/s 이상 미끄러지면 0.2초 안에 끈다
//        (방향을 반대로 켠 경우가 여기 걸린다)
//      - 무효 감지: 0.3m/s 이상 미끄러지는 개입이 3회 연속이면 끈다
//      - 구동/제동/홀딩/FAULT/E-Stop 명령이 오면 즉시 양보
//
//   5. updateDriveWatchdog 가 홀딩·안티롤백을 끄던 잠재 버그 수정.
//      H 홀딩도 2초 뒤 워치독이 목표를 0 으로 떨궈 램프와 싸우고 있었다.
//
//   6. STATUS 맨 뒤에 antiroll 상태 2필드 추가 (기존 파서 영향 없음)
//
//  v34 대비 변경 (v33 기준)  — 2026-08-29
//   엔코더 실측 반영 + 노이즈 차단 강화
//
//   1. ENCODER_CPR 436 → 163
//      아두이노 단독 측정: 955 카운트 / 4.78 m → counts_per_meter 199.8
//      회전당 = 199.8 × 둘레 0.817m = 163
//      (71 도 436 도 근거가 없었다. 533.1 을 믿고 계산한 값이었는데
//       그 533.1 자체가 틀렸다)
//
//   2. ENCODER_DEBOUNCE_US 200 → 2000   ★ 이게 더 중요하다
//      진짜 펄스 간격은 3단에서도 6.1ms 다. 200us 는 그 1/31 이라
//      사실상 아무 노이즈도 막지 못했다.
//
//        1단 0.229 m/s →  46 카운트/s → 간격 21.9 ms
//        2단 0.526 m/s → 105 카운트/s → 간격  9.5 ms
//        3단 0.820 m/s → 164 카운트/s → 간격  6.1 ms
//
//      2ms 로 올려도 3단 대비 여유가 3배다.
//      진짜 펄스는 하나도 안 잃고 브러시 노이즈만 10배 더 걸러낸다.
//
//      ※ 왜 필요한가: ROS 스택을 돌리며 정지/출발이 잦았던 세션에서
//        카운트율이 550개/초까지 올라갔다. 그 속도면 2.75 m/s 인데
//        이 차 최고속은 0.82 m/s 다. 대부분이 가짜 펄스였다.
//        모터 스위칭이 잦을수록 심해지므로 대회 주행에서 특히 위험하다.
//
//  v33 대비 변경 (v32 기준)  — 2026-08-28
//   제동을 고정 시간(150ms)에서 엔코더 피드백으로 바꿨다.
//
//   왜: 150ms 는 근거 없는 추측값이었다. 짧으면 안 서고, 길면 차가
//       멈춘 뒤에도 역토크가 남아 뒤로 밀린다. 코스팅 거리를 아직
//       재지 않아 적정값을 정할 근거가 없다.
//
//   어떻게: 역토크를 걸고 엔코더 펄스를 보다가, 일정 시간 창 안에서
//       움직임이 임계값 이하가 되면 "섰다"고 보고 즉시 끊는다.
//
//   ★ 안전 설계 — 엔코더를 믿되, 엔코더가 틀려도 시간이 막는다
//     A상만 세므로 방향을 알 수 없다. "멈췄나"는 알아도
//     "뒤로 가는 중인가"는 모른다. 게다가 저속에서 브러시 노이즈로
//     가짜 펄스가 생기면 "아직 움직인다"고 오판해 역토크가 안 끊긴다.
//     그래서 BRAKE_MAX_MS 상한을 둔다. 엔코더가 무슨 말을 하든
//     이 시간이 지나면 무조건 해제한다. 예전 150ms 고정값이 하던
//     안전 역할을 이 상한이 이어받는다.
//
//     반대로 BRAKE_MIN_MS 는 첫 표본의 노이즈로 즉시 해제되는 것을 막는다.
//
//   ⚠ 아래 임계값들은 코스팅 거리 실측 전까지는 잠정값이다.
//     실측 후 BRAKE_STOP_COUNTS 와 BRAKE_MAX_MS 를 다시 정할 것.
//
//  v32 대비 변경 (v31 기준)  — 2026-08-28
//   1. ENCODER_CPR 71 → 436 으로 확정.
//      바퀴 둘레 0.817m, counts_per_meter 533.1 실측에서 역산하면
//      바퀴 1회전 = 0.817 × 533.1 = 435.5 ≈ 436 카운트다.
//      71 이었을 때 STATUS 의 rpm 이 약 6배 크게 나왔다.
//      ※ odoCount 는 원시 펄스 누적이라 거리·속도·odom 에는 영향이 없다.
//         이 값은 rpm 표시에만 쓰인다.
//
//   2. 제동(B) 재진입 방어.
//      제동 펄스가 진행 중일 때 B 가 또 들어오면 예전에는 brakeStartMs 를
//      다시 찍어 펄스가 영원히 끝나지 않았다. 역토크가 계속 걸린 채로
//      유지되어 급정거를 걸었는데 차가 뒤로 가는 상황이 만들어진다.
//      상위(ROS 브릿지)에서도 1회만 보내도록 고쳤지만, 다른 팀이 직접
//      시리얼을 쓸 수도 있으므로 펌웨어에서도 막는다.
//
//  v31 대비 변경 (v30 기준)
//   조향 ADC 폐루프 정렬 추가. 포텐셔미터가 실제로 각도를 따라가는 경우에만
//   의미가 있다. 값 범위가 좁으면 CAL 이 거부한다.
//
//     CAL       좌우 끝단을 훑어 ADC 최소/최대를 재고 중앙값을 계산한다.
//               그 뒤 자동으로 중앙으로 이동한다.
//     AS<adc>   지정한 ADC 값으로 이동한다.  예: AS512
//     AC        마지막 CAL 로 구한 중앙(없으면 STEER_CENTER_ADC)으로 이동.
//
//   방향은 자동으로 찾는다. 조향을 조금 움직여 ADC 가 어느 쪽으로 가는지
//   보고 결정하므로 배선 극성을 몰라도 된다.
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

// ---- v31: ADC 폐루프 정렬 ----
//
//  고정 길이 펄스를 쓰면 한 걸음이 허용오차보다 커서 목표를 계속 지나친다.
//  (40ms 가 ADC 를 30 이상 움직이는데 오차 허용이 8 이면 영원히 발진한다)
//  그래서 '조향 1ms 가 ADC 를 몇 카운트 움직이는가'(gain)를 실측으로 학습하고,
//  남은 오차에 비례해 펄스 길이를 정한다.
constexpr long SEEK_PROBE_MS      = 60;    // gain 학습용 첫 펄스
constexpr long SEEK_MIN_PULSE_MS  = 30;    // 이보다 짧으면 모터가 안 움직인다
constexpr long SEEK_MAX_PULSE_MS  = 200;   // 한 번에 너무 크게 가지 않게
constexpr int  SEEK_MAX_PULSES    = 60;    // 안전 상한 (약 4초)
constexpr int  SEEK_MIN_PROGRESS  = 2;     // 이만큼도 안 변하면 정체
constexpr int  SEEK_STALL_LIMIT   = 4;     // 정체가 이어지면 방향 반전
constexpr int  SEEK_TOL_FLOOR     = 8;     // 허용오차 하한
constexpr int  CAL_MIN_SPAN       = 100;   // 범위가 이보다 좁으면 폐루프 불가

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
//  v33: 고정 펄스 길이를 없애고 엔코더 피드백으로 바꿨다.
//  (예전 주석: "BRAKE_PULSE_MS 를 크게 잡으면 역주행이 시작되므로 짧게
//   유지할 것" — 그 역할은 이제 BRAKE_MAX_MS 상한이 한다)
constexpr int           BRAKE_PULSE_PWM = 120;   // 제동 펄스 세기

// ---- v33: 엔코더 피드백 제동 ----
//
//  1카운트 = 1.9mm (counts_per_meter 533.1)
//  BRAKE_STOP_COUNTS 2 / BRAKE_WINDOW 50ms → 약 0.076 m/s 이하를 "정지"로 본다
//
constexpr unsigned long BRAKE_SAMPLE_MS      = 10;   // 표본 주기
constexpr uint8_t       BRAKE_WINDOW_SAMPLES = 5;    // 판정 창 = 50ms
constexpr long          BRAKE_STOP_COUNTS    = 2;    // 창 안 이동이 이 이하면 정지
constexpr unsigned long BRAKE_MIN_MS         = 30;   // 이 전에는 판정하지 않는다
constexpr unsigned long BRAKE_MAX_MS         = 300;  // ★ 무조건 해제 (안전 상한)

// ---- v29: 경사로 홀딩 ----
//  브레이크가 없어 정지 중 중력으로 밀린다. 램프 없이 지정 PWM 을 즉시
//  인가해 버틴다. 스톨 전류가 흐르므로 반드시 시간 제한을 둔다.
constexpr int           HOLD_PWM_MAX    = 90;    // 안전 상한
constexpr unsigned long HOLD_TIMEOUT_MS = 8000;  // 이 시간 지나면 자동 해제

// ---- v35: 경사로 자동 안티롤백 ----
//
//  H 홀딩은 사람이 "지금 버텨" 라고 눌러줘야 한다.
//  안티롤백은 밀리는 것을 엔코더로 알아채고 저 혼자 버틴다.
//
//  판정: PWM 이 0 인데 엔코더 펄스가 는다 = 차가 굴러가고 있다.
//  (A상 단상이라 부호가 없다. "어느 쪽" 인지는 AR1/AR2 로 사람이 준다)
//
//  1카운트 = 5.01mm (counts_per_meter 199.8)
constexpr bool          ANTIROLL_DEFAULT_ON   = false;  // 경사로 시험 후 true 고려
constexpr int           ANTIROLL_PWM_DEFAULT  = 70;     // HOLD_PWM_MAX 로 다시 잘린다
constexpr long          ANTIROLL_COUNTS       = 3;      // 약 15mm 밀리면 개입
constexpr unsigned long ANTIROLL_SAMPLE_MS    = 20;     // 감시 주기
constexpr unsigned long ANTIROLL_ARM_DELAY_MS = 400;    // 정지 후 코스팅이 끝나길 기다린다

//  토크 펄스는 제동(v33)과 같은 방식으로 엔코더가 끊는다.
//  잘 잡히면 짧게 끝나고, 잘 안 잡히면 상한까지 늘어난다 = 사실상 연속 홀딩.
constexpr unsigned long ANTIROLL_HOLD_MIN_MS  = 100;    // 이 전에는 정지 판정을 하지 않는다
constexpr unsigned long ANTIROLL_HOLD_MAX_MS  = 1200;   // ★ 1회 연속 유지 절대 상한
constexpr uint8_t       ANTIROLL_STILL_WINDOW = 3;      // 60ms 창
constexpr long          ANTIROLL_STILL_COUNTS = 1;      // 창 안 이동이 이 이하면 섰다

//  쿨다운은 직전 유지시간 이상으로 준다 → 듀티가 50% 를 못 넘는다 (스톨 발열)
constexpr unsigned long ANTIROLL_COOLDOWN_MS  = 150;
constexpr unsigned long ANTIROLL_SETTLE_MS    = 1500;   // 이만큼 조용하면 실패 카운터 해제

//  [자동 해제 판단]  토크를 넣는 동안 얼마나 미끄러지는가로 본다.
//    RUNAWAY : 방향을 반대로 켠 경우. 차가 스스로 가속해 달아난다 → 즉시 끈다
//    NO_EFFECT : 경사가 감당 밖이라 토크가 의미 없다 → 3회 연속이면 끈다
constexpr long          ANTIROLL_SLIP_CPS     = 60;   // 약 0.30 m/s 이상 미끄러지면 실패
constexpr long          ANTIROLL_RUNAWAY_CPS  = 200;  // 약 1.00 m/s 이상이면 폭주
constexpr unsigned long ANTIROLL_RUNAWAY_MIN_MS = 150; // 이 전에는 폭주 판정을 하지 않는다
constexpr uint8_t       ANTIROLL_ABORT_FAILS  = 3;
constexpr long          ANTIROLL_WARN_DRIFT   = 200;  // 감시 시작 후 1m 밀리면 경고만 낸다

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
//  ★ v34: 436 → 163 (아두이노 단독 실측).
//    955 카운트 / 4.78 m = counts_per_meter 199.8
//    회전당 = 199.8 × 둘레 0.817m = 163
constexpr float         ENCODER_CPR         = 163.0f;
//  ★ v34: 200 → 2000.  진짜 펄스 간격이 3단에서도 6.1ms 라
//    200us 는 아무것도 못 막았다. 2ms 여도 여유 3배.
//    ⚠ 최고속이 크게 올라가면 다시 낮출 것.
//      기준: 디바운스 < 펄스간격 / 2.  3단 6.1ms → 3ms 까지 안전.
constexpr uint32_t      ENCODER_DEBOUNCE_US = 2000;
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

// ---- v33: 제동 판정용 표본 링버퍼 ----
long          brakeWindow[BRAKE_WINDOW_SAMPLES];
uint8_t       brakeSampleIdx   = 0;
uint8_t       brakeSampleCount = 0;
unsigned long brakeLastSampleMs = 0;
bool          holdActive     = false;   // 경사로 홀딩 중
int           holdPwm        = 0;
unsigned long holdStartMs    = 0;

// ---- v35: 안티롤백 ----
bool          antiRollEnabled   = ANTIROLL_DEFAULT_ON;
bool          antiRollForward   = true;    // 버티는 방향 (AR1 전진 / AR2 후진)
int           antiRollPwm       = ANTIROLL_PWM_DEFAULT;

bool          antiRollArmed     = false;   // 기준점을 잡고 감시 중
long          antiRollSnapshot  = 0;       // 마지막 기준 펄스
long          antiRollArmPulse  = 0;       // 감시 시작 시점 펄스 (누적 밀림 계산용)
unsigned long antiRollStillMs   = 0;       // 정지 조건이 성립한 시각
unsigned long antiRollSampleMs  = 0;
bool          antiRollWarned    = false;

bool          antiRollHolding   = false;   // 지금 토크를 넣고 있다
unsigned long antiRollHoldStart = 0;
long          antiRollHoldPulse = 0;       // 개입 시작 시점 펄스 (미끄럼 계산용)
unsigned long antiRollHoldSample = 0;
long          antiRollWindow[ANTIROLL_STILL_WINDOW];
uint8_t       antiRollWinIdx    = 0;
uint8_t       antiRollWinCount  = 0;
unsigned long antiRollCooldown  = 0;       // 이 시각 전에는 다시 안 문다
uint8_t       antiRollFails     = 0;       // 연속 미끄럼 횟수
uint16_t      antiRollEngages   = 0;       // 총 개입 횟수 (기록용)

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

// ---- v31: 폐루프 정렬 상태 ----
enum SeekPhase : uint8_t {
  SEEK_OFF = 0,
  SEEK_TO_TARGET,      // 목표 ADC 로 이동
  CAL_TO_LEFT,         // 좌 끝단 탐색
  CAL_TO_RIGHT,        // 우 끝단 탐색
  CAL_TO_CENTER        // 계산된 중앙으로 이동
};
SeekPhase seekPhase      = SEEK_OFF;
int   seekTargetAdc      = STEER_CENTER_ADC;
int8_t seekDir           = 1;      // 현재 시도 중인 방향
int   seekLastAdc        = 0;
int   seekPulses         = 0;
int   seekStall          = 0;
bool  seekDirKnown       = false;  // ADC 가 증가하는 방향을 알아냈는가
int8_t seekDirForPlus    = 1;      // ADC 를 키우는 조향 방향
int   calAdcMin          = 0;
int   calAdcMax          = 0;
int   calCenterAdc       = STEER_CENTER_ADC;
bool  calDone            = false;
float seekGain           = 0.0f;   // ADC 카운트 / 조향 ms (학습값)
int   seekTol            = SEEK_TOL_FLOOR;
long  seekLastPulseMs    = 0;
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

// ==========================================================
// v31. 조향 ADC 폐루프 정렬
//
//  포텐셔미터가 실제 조향각을 따라가는 경우에만 쓸 수 있다.
//  배선 극성(어느 방향이 ADC 를 키우는지)을 모르므로,
//  한 번 움직여 보고 ADC 변화를 관찰해 스스로 알아낸다.
//
//  한계에 닿거나 진전이 없으면 방향을 뒤집고,
//  그래도 안 되면 포기하고 이유를 출력한다.
// ==========================================================

void startSteer(int8_t sign, long ms);
void cancelSteer();

void seekAbort(const __FlashStringHelper* reason)
{
  seekPhase = SEEK_OFF;
  cancelSteer();
  Serial.print(F("SEEK_ABORT,"));
  Serial.print(reason);
  Serial.print(',');
  Serial.println(currentSteerAdc);
}

void seekBegin(SeekPhase phase, int target)
{
  // 포텐셔미터가 전기적으로 살아있는지 확인한다.
  // 0 이나 1023 에 붙어 있으면 배선이 끝단에 직결된 것이다.
  if (currentSteerAdc < POT_ELECTRICAL_MIN ||
      currentSteerAdc > POT_ELECTRICAL_MAX) {
    Serial.print(F("SEEK_ERR,POT_OUT_OF_RANGE,"));
    Serial.println(currentSteerAdc);
    Serial.println(F("  A0 가 5V 나 GND 에 직결된 것으로 보인다."));
    Serial.println(F("  포텐셔미터 가운데 탭이 A0 인지 확인할 것."));
    return;
  }
  cancelSteer();
  seekPhase     = phase;
  seekTargetAdc = target;
  seekTol       = SEEK_TOL_FLOOR;
  seekLastAdc   = currentSteerAdc;
  seekPulses    = 0;
  seekStall     = 0;
  seekDir       = seekDirKnown
                  ? (int8_t)((target > currentSteerAdc) ? seekDirForPlus
                                                        : -seekDirForPlus)
                  : (int8_t)1;
  lastDriveCommandMs = millis();
}

//  한 번의 펄스가 끝난 뒤 호출된다. 진전을 보고 다음 행동을 정한다.
void seekEvaluate()
{
  int adc  = currentSteerAdc;
  int diff = adc - seekLastAdc;

  // 방향과 gain 을 이번 움직임으로 학습한다.
  if (abs(diff) >= SEEK_MIN_PROGRESS && seekLastPulseMs >= SEEK_MIN_PULSE_MS) {
    float g = (float)abs(diff) / (float)seekLastPulseMs;
    seekGain = (seekGain <= 0.0f) ? g : (seekGain * 0.7f + g * 0.3f);

    // 허용오차는 '최소 펄스로 움직이는 양'보다 커야 수렴한다.
    int floorTol = (int)(seekGain * SEEK_MIN_PULSE_MS * 0.6f);
    seekTol = max(SEEK_TOL_FLOOR, floorTol);

    if (!seekDirKnown) {
      seekDirForPlus = (diff > 0) ? seekDir : (int8_t)(-seekDir);
      seekDirKnown   = true;
      Serial.print(F("SEEK_DIR_LEARNED,"));
      Serial.print(seekDirForPlus > 0 ? F("RIGHT_INCREASES")
                                      : F("LEFT_INCREASES"));
      Serial.print(F(",gain="));
      Serial.print(seekGain, 3);
      Serial.print(F(",tol="));
      Serial.println(seekTol);
    }
  }

  int error = seekTargetAdc - adc;

  // 끝단 탐색은 더 못 갈 때까지 밀어붙인다.
  if (seekPhase == CAL_TO_LEFT || seekPhase == CAL_TO_RIGHT) {
    if (abs(diff) < SEEK_MIN_PROGRESS) {
      seekStall++;
      if (seekStall >= SEEK_STALL_LIMIT) {
        if (seekPhase == CAL_TO_LEFT) {
          calAdcMin = adc;
          Serial.print(F("CAL_LEFT,"));  Serial.println(adc);
          seekPhase  = CAL_TO_RIGHT;
          seekDir    = (int8_t)(-seekDir);
          seekStall  = 0;
          seekPulses = 0;
        } else {
          calAdcMax = adc;
          Serial.print(F("CAL_RIGHT,")); Serial.println(adc);

          int lo = min(calAdcMin, calAdcMax);
          int hi = max(calAdcMin, calAdcMax);
          int span = hi - lo;
          Serial.print(F("CAL_SPAN,")); Serial.println(span);

          if (span < CAL_MIN_SPAN) {
            calDone = false;
            seekPhase = SEEK_OFF;
            cancelSteer();
            Serial.println(F("CAL_FAIL,SPAN_TOO_SMALL"));
            Serial.println(F("  포텐셔미터가 조향을 제대로 따라가지 않는다."));
            Serial.println(F("  링키지를 확인하거나 시간 기반 조향을 쓸 것."));
            return;
          }
          calCenterAdc = (lo + hi) / 2;
          calDone      = true;
          Serial.print(F("CAL_CENTER,")); Serial.println(calCenterAdc);
          seekBegin(CAL_TO_CENTER, calCenterAdc);
          return;
        }
      }
    } else {
      seekStall = 0;
    }
    seekLastAdc = adc;
    return;
  }

  // 목표값 추종
  if (abs(error) <= seekTol) {
    seekPhase = SEEK_OFF;
    cancelSteer();
    // 이 자리를 조향 중앙으로 기록한다 (시간 기반 좌표계와 동기화)
    steerNetMs    = 0;
    steerTargetMs = 0;
    Serial.print(F("SEEK_OK,"));
    Serial.print(adc);
    Serial.print(F(",err="));
    Serial.println(error);
    return;
  }

  if (abs(diff) < SEEK_MIN_PROGRESS) {
    seekStall++;
    if (seekStall >= SEEK_STALL_LIMIT) {
      seekDir   = (int8_t)(-seekDir);   // 방향이 틀렸거나 끝단에 닿았다
      seekStall = 0;
      Serial.println(F("SEEK_REVERSE"));
    }
  } else {
    seekStall = 0;
    if (seekDirKnown) {
      seekDir = (error > 0) ? seekDirForPlus : (int8_t)(-seekDirForPlus);
    }
  }
  seekLastAdc = adc;
}

void updateSeek(unsigned long now)
{
  if (seekPhase == SEEK_OFF) {
    return;
  }
  if (steerRunning) {
    return;                       // 펄스가 아직 진행 중
  }

  // 펄스가 막 끝났으면 결과를 평가한다.
  if (seekPulses > 0) {
    seekEvaluate();
    if (seekPhase == SEEK_OFF) {
      return;
    }
  }

  if (++seekPulses > SEEK_MAX_PULSES) {
    seekAbort(F("MAX_PULSES"));
    return;
  }

  lastDriveCommandMs = now;       // 워치독이 끼어들지 않게

  // 남은 오차에 비례해 펄스 길이를 정한다. gain 을 모르면 탐침 펄스.
  long pulse;
  if (seekGain > 0.0f && seekPhase == SEEK_TO_TARGET) {
    long need = (long)((float)abs(seekTargetAdc - currentSteerAdc) / seekGain);
    pulse = constrain(need, SEEK_MIN_PULSE_MS, SEEK_MAX_PULSE_MS);
  } else {
    pulse = SEEK_PROBE_MS;        // 끝단 탐색이나 학습 전에는 고정
  }
  seekLastPulseMs = pulse;
  startSteer(seekDir, pulse);

  // startSteer 가 한계로 거부하면 방향을 뒤집는다.
  if (!steerRunning) {
    seekDir = (int8_t)(-seekDir);
    seekStall++;
    if (seekStall >= SEEK_STALL_LIMIT) {
      seekAbort(F("BLOCKED"));
    }
  }
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
// 12-b. v35 경사로 자동 안티롤백
//
//  [왜 필요한가]
//   이 차에는 기계식 브레이크가 없다. 경사로에 세우면 그냥 밀린다.
//   v29 의 H 홀딩은 사람이 눌러줘야 해서 자율주행 중에는 못 쓴다.
//
//  [어떻게 아는가]
//   PWM 이 0 인데 엔코더 펄스가 늘고 있으면 차가 굴러가는 중이다.
//   우리 엔코더는 A상 단상이라 방향 부호가 없다. 그래서 "어느 쪽으로
//   미는가" 는 AR1(전진 버팀) / AR2(후진 버팀) 로 사람이 알려준다.
//
//  [왜 계속 물고 있지 않는가]
//   스톨 상태는 전류가 그대로 흐른다. 모터와 MD30C 가 탄다.
//   그래서 150ms 짧게 물고 놓고, 또 밀리면 다시 문다. 작년 방식과 같다.
//
//  [잘못 켰을 때]
//   AR1 을 내리막에서 켜면 차를 스스로 앞으로 밀어낸다.
//   개입이 ANTIROLL_ABORT_ENGAGES 회 이어지면 방향이 틀린 것으로 보고
//   스스로 꺼진다. 그래도 방향은 사람이 확인하고 켤 것.
// ==========================================================
void releaseAntiRoll(bool alsoDisarm)
{
  if (antiRollHolding) {
    applyDrivePwm(0);
    targetDrivePwm  = 0;
    antiRollHolding = false;
  }
  if (alsoDisarm) {
    antiRollArmed   = false;
    antiRollStillMs = 0;
    antiRollFails   = 0;
    antiRollWarned  = false;
  }
}

//  개입 종료 — 토크를 놓고 기준점을 다시 잡는다.
//
//  why 는 왜 끝났는지. 실차에서 어느 쪽으로 끝나는지 봐야
//  ANTIROLL_PWM / SLIP 임계값을 조정할 수 있으므로 반드시 남긴다.
void endAntiRollHold(unsigned long now, const __FlashStringHelper *why)
{
  unsigned long elapsed = now - antiRollHoldStart;
  if (elapsed == 0) {
    elapsed = 1;
  }

  //  토크를 넣는 동안 얼마나 미끄러졌는가 = 이 개입이 먹혔는가
  long slip = readEncoderPulseAtomic() - antiRollHoldPulse;
  if (slip < 0) {
    slip = -slip;
  }
  long slipCps = (slip * 1000L) / (long)elapsed;

  applyDrivePwm(0);
  targetDrivePwm  = 0;
  antiRollHolding = false;

  //  쿨다운을 직전 유지시간 이상으로 준다 → 스톨 듀티가 50% 를 못 넘는다.
  unsigned long cool = elapsed;
  if (cool < ANTIROLL_COOLDOWN_MS) {
    cool = ANTIROLL_COOLDOWN_MS;
  }
  antiRollCooldown = now + cool;

  //  놓은 시점을 새 기준점으로 삼는다. 안 그러면 개입 중에 돈 만큼이
  //  그대로 "또 밀렸다" 로 읽혀 무한히 물게 된다.
  antiRollSnapshot = readEncoderPulseAtomic();
  antiRollStillMs  = now;

  if (slipCps > ANTIROLL_SLIP_CPS) {
    if (antiRollFails < 255) {
      antiRollFails++;
    }
  }
  else {
    antiRollFails = 0;
  }

  Serial.print(F("ANTIROLL_OFF,"));
  Serial.print(why);
  Serial.print(',');
  Serial.print(elapsed);
  Serial.print(F(",SLIP,"));
  Serial.print(slipCps);
  Serial.print(F(",FAIL,"));
  Serial.println(antiRollFails);

  //  경사가 감당 밖이다. 계속 물어봐야 모터만 상한다.
  if (antiRollFails >= ANTIROLL_ABORT_FAILS) {
    antiRollEnabled = false;
    releaseAntiRoll(true);
    Serial.println(F("ANTIROLL_ABORT,NO_EFFECT"));
  }
}

//  방향을 반대로 켰다. 차가 스스로 가속해 달아나고 있다. 즉시 끈다.
void abortAntiRollRunaway(unsigned long now, long slipCps)
{
  applyDrivePwm(0);
  targetDrivePwm   = 0;
  antiRollHolding  = false;
  antiRollEnabled  = false;
  antiRollArmed    = false;
  antiRollStillMs  = 0;
  antiRollCooldown = now;

  Serial.print(F("ANTIROLL_ABORT,RUNAWAY,"));
  Serial.println(slipCps);
}

void updateAntiRoll(unsigned long now)
{
  if (!antiRollEnabled) {
    return;
  }

  // ---- [1] 개입 중 ----
  //  ★ 이 검사가 [2] 보다 먼저여야 한다.
  //    개입 중에는 우리가 targetDrivePwm 을 올려두므로, 순서가 반대면
  //    "누가 주행 중" 으로 스스로를 오인해 다음 루프에서 바로 놓아버린다.
  //    주행/제동/홀딩 명령은 들어오는 즉시 releaseAntiRoll() 을 부르므로
  //    여기서 다시 확인하지 않아도 선점된다.
  if (antiRollHolding) {
    unsigned long elapsed = now - antiRollHoldStart;

    // (1-a) 절대 상한. 엔코더가 무슨 말을 하든 여기서 무조건 놓는다.
    if (elapsed >= ANTIROLL_HOLD_MAX_MS) {
      endAntiRollHold(now, F("MAXTIME"));
      return;
    }

    // 램프가 홀딩 PWM 을 갉아먹지 않도록 목표를 계속 고정한다.
    targetDrivePwm = antiRollPwm;

    if (now - antiRollHoldSample < ANTIROLL_SAMPLE_MS) {
      return;
    }
    antiRollHoldSample = now;

    long pulse = readEncoderPulseAtomic();
    long slip  = pulse - antiRollHoldPulse;
    if (slip < 0) {
      slip = -slip;
    }

    // (1-b) 폭주 감지 — 상한을 기다리지 않고 즉시 끊는다.
    //   방향을 반대로 켜면 차가 가속한다. 이걸 1.2초나 두면 안 된다.
    if (elapsed >= ANTIROLL_RUNAWAY_MIN_MS) {
      long cps = (slip * 1000L) / (long)elapsed;
      if (cps > ANTIROLL_RUNAWAY_CPS) {
        abortAntiRollRunaway(now, cps);
        return;
      }
    }

    // (1-c) 정지 판정 — 제동(v33)과 같은 링버퍼 방식
    if (elapsed < ANTIROLL_HOLD_MIN_MS) {
      return;
    }
    antiRollWindow[antiRollWinIdx] = pulse;
    antiRollWinIdx = (uint8_t)((antiRollWinIdx + 1) % ANTIROLL_STILL_WINDOW);
    if (antiRollWinCount < ANTIROLL_STILL_WINDOW) {
      antiRollWinCount++;
      return;                       // 창이 아직 안 찼다
    }
    long oldest = antiRollWindow[antiRollWinIdx];
    long moved  = pulse - oldest;
    if (moved < 0) {
      moved = -moved;
    }
    if (moved <= ANTIROLL_STILL_COUNTS) {
      endAntiRollHold(now, F("STOPPED"));
    }
    return;
  }

  // ---- [2] 다른 기능이 구동을 쓰고 있으면 양보한다 ----
  //  제동 펄스 / H 홀딩 / 방향 전환 대기 / 실제 주행 명령
  if (brakeActive || holdActive || directionChangePending || targetDrivePwm > 0) {
    antiRollArmed   = false;
    antiRollStillMs = 0;
    antiRollFails   = 0;
    return;
  }

  // ---- [3] 완전히 서 있는가 ----
  if (currentDrivePwm != 0) {
    antiRollArmed   = false;
    antiRollStillMs = 0;
    return;
  }

  // ---- [4] 정지 직후 안정화를 기다렸다가 기준점을 잡는다 ----
  if (!antiRollArmed) {
    if (antiRollStillMs == 0) {
      antiRollStillMs = now;
      return;
    }
    if (now - antiRollStillMs < ANTIROLL_ARM_DELAY_MS) {
      return;              // 코스팅이 끝나기를 기다린다
    }
    antiRollSnapshot = readEncoderPulseAtomic();
    antiRollArmPulse = antiRollSnapshot;
    antiRollArmed    = true;
    antiRollWarned   = false;
    antiRollSampleMs = now;
    return;
  }

  // ---- [5] 감시 ----
  if (now - antiRollSampleMs < ANTIROLL_SAMPLE_MS) {
    return;
  }
  antiRollSampleMs = now;

  long pulse = readEncoderPulseAtomic();

  //  감시 시작 후 누적으로 얼마나 밀렸는지 — 끄지는 않고 경고만 낸다.
  //  (여기서 꺼버리면 정작 필요한 순간에 손을 놓는 셈이 된다)
  long drift = pulse - antiRollArmPulse;
  if (drift < 0) {
    drift = -drift;
  }
  if (!antiRollWarned && drift >= ANTIROLL_WARN_DRIFT) {
    antiRollWarned = true;
    Serial.print(F("ANTIROLL_WARN,DRIFT,"));
    Serial.println(drift);
  }

  long moved = pulse - antiRollSnapshot;
  if (moved < 0) {
    moved = -moved;                       // 카운터가 리셋된 경우
  }

  if (moved < ANTIROLL_COUNTS) {
    // 조용하다. 충분히 오래 조용하면 실패 카운터를 푼다.
    if (antiRollFails > 0 && (now - antiRollStillMs) >= ANTIROLL_SETTLE_MS) {
      antiRollFails = 0;
      Serial.println(F("ANTIROLL_SETTLED"));
    }
    return;
  }

  // ---- [6] 밀림 확정 ----
  if (now < antiRollCooldown) {
    return;                              // 스톨 듀티 제한
  }

  setDriveDirection(antiRollForward);
  applyDrivePwm(antiRollPwm);
  targetDrivePwm     = antiRollPwm;
  antiRollHolding    = true;
  antiRollHoldStart  = now;
  antiRollHoldPulse  = pulse;
  antiRollHoldSample = now;
  antiRollWinIdx     = 0;
  antiRollWinCount   = 0;
  if (antiRollEngages < 65535) {
    antiRollEngages++;
  }

  Serial.print(F("ANTIROLL_ON,"));
  Serial.print(moved);
  Serial.print(',');
  Serial.print(antiRollForward ? F("FWD") : F("REV"));
  Serial.print(',');
  Serial.println(antiRollEngages);
}

//  AR1 / AR2 / AR0
void requestAntiRoll(int mode)
{
  lastDriveCommandMs = millis();

  if (mode == 0) {
    antiRollEnabled = false;
    releaseAntiRoll(true);
    Serial.println(F("ANTIROLL_MODE,OFF"));
    return;
  }

  // 달리는 중에는 켜지 않는다. 켜자마자 오판할 수 있다.
  if (currentDrivePwm > 0 || targetDrivePwm > 0) {
    Serial.println(F("ANTIROLL_REJECTED,MOVING"));
    return;
  }

  antiRollForward  = (mode == 1);
  antiRollEnabled  = true;
  antiRollArmed    = false;
  antiRollStillMs  = 0;
  antiRollFails    = 0;
  antiRollWarned   = false;
  antiRollCooldown = 0;

  Serial.print(F("ANTIROLL_MODE,"));
  Serial.print(antiRollForward ? F("FWD") : F("REV"));
  Serial.print(F(",PWM,"));
  Serial.println(antiRollPwm);
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
  antiRollHolding        = false;   // v35: 토크를 물고 있으면 놓는다
  antiRollArmed          = false;
  antiRollStillMs        = 0;
  applyDrivePwm(0);
  cancelSteer();
}

// ==========================================================
// 13-b. v29 급정거 — 다이내믹 브레이킹
//
//  감속 램프(50ms 당 -10)를 건너뛰고 즉시 세운다.
//  DIR 을 반대로 두고 짧은 역토크 펄스를 준 뒤 PWM 0 으로 떨어뜨린다.
//  펄스가 길면 역주행이 시작된다. v33 부터는 엔코더가 정지를 감지해
//  끊고, 그래도 안 끊기면 BRAKE_MAX_MS 가 강제로 끝낸다.
//
//  주의: 모터·기어·드라이버에 부담이 간다. 비상시에만 쓴다.
// ==========================================================
void requestBrake()
{
  lastDriveCommandMs = millis();

  // ★ v32: 제동 펄스가 진행 중이면 타이머를 다시 찍지 않는다.
  //
  //   예전에는 여기서 그냥 진행해 brakeStartMs 가 갱신되었다.
  //   상위가 B 를 제동 시간보다 짧은 주기로 반복 전송하면 updateBrake()
  //   의 종료 조건이 영원히 성립하지 않는다. 역토크 무한 유지 = 후진.
  //
  //   워치독은 위에서 이미 먹였으므로 그냥 돌아가면 된다.
  //   (응답을 찍으면 10Hz 반복 시 시리얼이 넘친다 — 조용히 넘긴다)
  if (brakeActive) {
    return;
  }

  // 이미 서 있으면 펄스 없이 끝낸다 (불필요한 역토크 방지)
  if (currentDrivePwm == 0 && !brakeActive) {
    targetDrivePwm         = 0;
    pendingDrivePwm        = 0;
    directionChangePending = false;
    directionHoldActive    = false;
    holdActive             = false;
    releaseAntiRoll(true);          // v35
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
  releaseAntiRoll(true);            // v35

  // 역토크 펄스: 현재 진행 방향의 반대로 DIR 을 두고 짧게 인가
  applyDrivePwm(0);
  digitalWrite(DIR_DRIVE_FRONT, driveForward ? DRIVE_FRONT_REV : DRIVE_FRONT_FWD);
  digitalWrite(DIR_DRIVE_REAR,  driveForward ? DRIVE_REAR_REV  : DRIVE_REAR_FWD);
  applyDrivePwm(BRAKE_PULSE_PWM);

  brakeActive       = true;
  brakeStartMs      = millis();
  brakeLastSampleMs = brakeStartMs;
  brakeSampleIdx    = 0;
  brakeSampleCount  = 0;

  Serial.print(F("BRAKE_OK,MAX"));
  Serial.println(BRAKE_MAX_MS);
}

// ==========================================================
//  v33: 제동 종료 — 한 곳에서만 끝낸다
//
//  DIR 을 원래 진행 방향으로 되돌리고 PWM 을 0 으로 떨어뜨린다.
//  why 는 왜 끝났는지 (STOPPED / TIMEOUT). 실차에서 어느 쪽으로
//  끝나는지 봐야 임계값을 조정할 수 있으므로 반드시 남긴다.
// ==========================================================
void endBrake(const __FlashStringHelper *why, unsigned long elapsed)
{
  applyDrivePwm(0);
  targetDrivePwm = 0;
  setDriveDirection(driveForward);
  brakeActive      = false;
  brakeSampleIdx   = 0;
  brakeSampleCount = 0;

  Serial.print(F("BRAKE_DONE,"));
  Serial.print(why);
  Serial.print(',');
  Serial.println(elapsed);
}

void updateBrake(unsigned long now)
{
  if (!brakeActive) {
    return;
  }

  unsigned long elapsed = now - brakeStartMs;

  // ---- [1] 안전 상한 ----
  //  엔코더가 무엇을 말하든 여기서 무조건 끝낸다.
  //  브러시 노이즈로 가짜 펄스가 계속 생기거나, 차가 이미 반대로
  //  움직이기 시작했을 때 역토크가 무한히 유지되는 것을 막는다.
  if (elapsed >= BRAKE_MAX_MS) {
    endBrake(F("TIMEOUT"), elapsed);
    return;
  }

  // ---- [2] 최소 제동 시간 ----
  //  첫 표본 하나의 노이즈로 즉시 해제되면 제동이 아예 안 걸린다.
  if (elapsed < BRAKE_MIN_MS) {
    return;
  }

  // ---- [3] 표본 수집 ----
  if (now - brakeLastSampleMs < BRAKE_SAMPLE_MS) {
    return;
  }
  brakeLastSampleMs = now;

  //  odoCount 는 ENCODER_UPDATE_MS(100ms)마다 갱신되어 여기엔 너무 느리다.
  //  ISR 이 올리는 원시 카운터를 직접 읽는다.
  long pulse = readEncoderPulseAtomic();

  brakeWindow[brakeSampleIdx] = pulse;
  brakeSampleIdx = (uint8_t)((brakeSampleIdx + 1) % BRAKE_WINDOW_SAMPLES);
  if (brakeSampleCount < BRAKE_WINDOW_SAMPLES) {
    brakeSampleCount++;
    return;                       // 창이 아직 안 찼다
  }

  // ---- [4] 정지 판정 ----
  //  링버퍼가 가득 찼으므로 "다음에 덮어쓸 자리" 가 가장 오래된 표본이다.
  long oldest = brakeWindow[brakeSampleIdx];
  long moved  = pulse - oldest;
  if (moved < 0) {
    moved = -moved;
  }
  if (moved <= BRAKE_STOP_COUNTS) {
    endBrake(F("STOPPED"), elapsed);
  }
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
    releaseAntiRoll(true);          // v35
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
  releaseAntiRoll(true);            // v35: 사람 명령이 우선

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
  releaseAntiRoll(true);            // v35: 주행 명령이 안티롤백보다 우선

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
  // ★ v35: 홀딩·안티롤백은 펌웨어가 스스로 거는 토크다.
  //   둘 다 자체 시간 상한이 있으므로 워치독이 끼어들면 안 된다.
  //   (예전에는 H 홀딩 2초 뒤 워치독이 목표를 0 으로 떨궈
  //    updateHold 와 매 루프 싸웠다 — 잠재 버그였다)
  if (holdActive || antiRollHolding) {
    return;
  }

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
  Serial.print(sRaw);

  // v35: 안티롤백 (또 맨 뒤에만 추가한다. 기존 파서는 안 깨진다)
  //   antiroll_state   : 0 꺼짐 / 1 감시중 / 2 개입중
  //   antiroll_engages : 총 개입 횟수 (경사로에서 몇 번 물었나)
  Serial.print(',');
  Serial.print(antiRollHolding ? 2 : (antiRollEnabled ? 1 : 0));
  Serial.print(',');
  Serial.println(antiRollEngages);
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
  Serial.println(F("-- v31 --"));
  Serial.println(F("CAL   sweep both ends, find center, go there"));
  Serial.println(F("AC    go to calibrated center"));
  Serial.println(F("AS512 go to ADC 512"));
  Serial.println(F("SEEKOFF abort seeking"));
  Serial.println(F("-- v35 --"));
  Serial.println(F("AR1 anti-rollback hold FWD (uphill)"));
  Serial.println(F("AR2 anti-rollback hold REV (downhill)"));
  Serial.println(F("AR0 anti-rollback off"));
  Serial.println(F("ARP70 anti-rollback hold pwm"));
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
  antiRollFails      = 0;            // v35: 실패 카운터도 푼다
  antiRollCooldown   = 0;
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

  // v31: 조향 ADC 폐루프
  if (strcmp(command, "CAL") == 0) {
    Serial.println(F("CAL_START,좌우 끝단 탐색"));
    seekDirKnown = false;
    calDone      = false;
    seekBegin(CAL_TO_LEFT, 0);
    return;
  }
  if (strcmp(command, "AC") == 0) {
    int target = calDone ? calCenterAdc : STEER_CENTER_ADC;
    Serial.print(F("SEEK_START,")); Serial.println(target);
    seekBegin(SEEK_TO_TARGET, target);
    return;
  }
  if (command[0] == 'A' && command[1] == 'S' && length >= 3) {
    long value;
    if (!parseLongStrict(command + 2, value)) {
      Serial.println(F("SEEK_ERR,BAD_VALUE"));
      return;
    }
    value = constrain(value, 0, 1023);
    Serial.print(F("SEEK_START,")); Serial.println(value);
    seekBegin(SEEK_TO_TARGET, (int)value);
    return;
  }
  // v35: 경사로 자동 안티롤백
  //  ⚠ 조향 조그 'A200' 보다 반드시 앞에서 걸러야 한다.
  if (strcmp(command, "AR0") == 0) { requestAntiRoll(0); return; }
  if (strcmp(command, "AR1") == 0) { requestAntiRoll(1); return; }
  if (strcmp(command, "AR2") == 0) { requestAntiRoll(2); return; }
  if (command[0] == 'A' && command[1] == 'R' && command[2] == 'P' && length >= 4) {
    long value;
    if (!parseLongStrict(command + 3, value)) {
      Serial.println(F("ANTIROLL_ERR,BAD_VALUE"));
      return;
    }
    antiRollPwm = constrain((int)value, 0, HOLD_PWM_MAX);
    Serial.print(F("ANTIROLL_PWM,"));
    Serial.println(antiRollPwm);
    return;
  }

  if (strcmp(command, "SEEKOFF") == 0) {
    seekPhase = SEEK_OFF;
    cancelSteer();
    Serial.println(F("SEEK_OFF"));
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
    previousPulse    = 0;
    odoCount         = 0;
    antiRollSnapshot = 0;          // v35: 기준점도 같이 옮긴다
    antiRollArmPulse = 0;
    antiRollArmed    = false;
    antiRollStillMs  = 0;
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

  Serial.println(F("MCU_BOOT,v35"));
  Serial.print(F("CENTER_ADC,"));   Serial.println(STEER_CENTER_ADC);
  Serial.print(F("CURRENT_ADC,"));  Serial.println(currentSteerAdc);
  Serial.print(F("STEER_LIMIT,"));  Serial.println(steerLimitMs);
  Serial.print(F("WATCHDOG_MS,"));  Serial.println(DRIVE_COMMAND_TIMEOUT_MS);
  Serial.print(F("BRAKE_PWM,"));    Serial.println(BRAKE_PULSE_PWM);
  Serial.print(F("BRAKE_WINDOW,")); Serial.print(BRAKE_WINDOW_SAMPLES * BRAKE_SAMPLE_MS);
  Serial.print(F("ms,STOP<="));     Serial.print(BRAKE_STOP_COUNTS);
  Serial.print(F(",MIN,"));         Serial.print(BRAKE_MIN_MS);
  Serial.print(F(",MAX,"));         Serial.println(BRAKE_MAX_MS);
  Serial.print(F("HOLD_PWM_MAX,")); Serial.println(HOLD_PWM_MAX);
  Serial.print(F("ANTIROLL,"));     Serial.print(antiRollEnabled ? F("ON") : F("OFF"));
  Serial.print(F(",PWM,"));         Serial.print(antiRollPwm);
  Serial.print(F(",COUNTS,"));      Serial.print(ANTIROLL_COUNTS);
  Serial.print(F(",HOLD,"));        Serial.print(ANTIROLL_HOLD_MIN_MS);
  Serial.print('~');                Serial.println(ANTIROLL_HOLD_MAX_MS);
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
    antiRollHolding        = false;   // v35
    antiRollArmed          = false;   // v35: 복귀 후 다시 기준점을 잡게 한다
    antiRollStillMs        = 0;
    seekPhase              = SEEK_OFF; // v31
  }
  else {
    updateBrake(now);                 // v29: 램프보다 먼저
    updateHold(now);                  // v29
    updateAntiRoll(now);              // v35: 홀딩 다음. 램프보다는 먼저
    updateSeek(now);                  // v31: 조향 폐루프
    updateDriveWatchdog(now);
    updateDriveController(now);
    updateSteering(now);
  }

  updateEncoder(now);
  updateSystemState();
  updateTelemetry(now);
}
