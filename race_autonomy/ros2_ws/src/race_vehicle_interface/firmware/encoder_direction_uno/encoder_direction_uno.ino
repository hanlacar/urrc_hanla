#include <avr/wdt.h>
#include <util/atomic.h>

constexpr uint8_t ENCODER_A_PIN = 2;
constexpr uint8_t ENCODER_B_PIN = 3;
constexpr int8_t ENCODER_SIGN = 1;
constexpr unsigned long PUBLISH_INTERVAL_MS = 20;
constexpr unsigned long STOP_TIMEOUT_MS = 200;

volatile int32_t encoderCount = 0;
volatile int8_t lastDirection = 0;
volatile uint8_t previousAB = 0;
volatile uint32_t lastEdgeMicros = 0;
volatile uint32_t invalidTransitions = 0;
volatile bool seenValidTransition = false;

int32_t lastPublishedCount = 0;
unsigned long lastPublishMs = 0;

// Index: previous AB in bits 3:2, current AB in bits 1:0.
const int8_t QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

uint8_t readAB() {
  return (digitalRead(ENCODER_A_PIN) << 1) |
         digitalRead(ENCODER_B_PIN);
}

void onEncoderChange() {
  const uint8_t currentAB = readAB();
  const uint8_t transition = (previousAB << 2) | currentAB;
  int8_t step = QUADRATURE_TABLE[transition] * ENCODER_SIGN;
  if (step != 0) {
    encoderCount += step;
    lastDirection = step > 0 ? 1 : -1;
    lastEdgeMicros = micros();
    seenValidTransition = true;
  } else if (currentAB != previousAB) {
    // Both channels changed between samples: noise or missed transition.
    invalidTransitions++;
  }
  previousAB = currentAB;
}

void setup() {
  // High-impedance taps: the existing Mega/encoder circuit owns pull-ups.
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  previousAB = readAB();
  lastEdgeMicros = micros();
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onEncoderChange, CHANGE);
  Serial.begin(115200);
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  const unsigned long nowMs = millis();
  if (nowMs - lastPublishMs < PUBLISH_INTERVAL_MS) {
    return;
  }
  lastPublishMs = nowMs;

  int32_t countSnapshot;
  int8_t directionSnapshot;
  uint32_t edgeMicrosSnapshot;
  uint32_t invalidSnapshot;
  bool signalValidSnapshot;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    countSnapshot = encoderCount;
    directionSnapshot = lastDirection;
    edgeMicrosSnapshot = lastEdgeMicros;
    invalidSnapshot = invalidTransitions;
    signalValidSnapshot = seenValidTransition;
  }

  const int32_t delta = countSnapshot - lastPublishedCount;
  lastPublishedCount = countSnapshot;
  const uint32_t edgeAgeMs = (micros() - edgeMicrosSnapshot) / 1000UL;
  if (edgeAgeMs >= STOP_TIMEOUT_MS) {
    directionSnapshot = 0;
  }

  Serial.print("E,");
  Serial.print(countSnapshot);
  Serial.print(',');
  Serial.print(delta);
  Serial.print(',');
  Serial.print(directionSnapshot);
  Serial.print(',');
  Serial.print(edgeAgeMs);
  Serial.print(',');
  Serial.print(invalidSnapshot);
  Serial.print(',');
  Serial.println(signalValidSnapshot ? 1 : 0);
}
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

uint8_t readAB() {
  return (digitalRead(ENCODER_A_PIN) << 1) |
         digitalRead(ENCODER_B_PIN);
}

void onEncoderChange() {
  const uint8_t currentAB = readAB();
  const uint8_t transition = (previousAB << 2) | currentAB;
  int8_t step = QUADRATURE_TABLE[transition] * ENCODER_SIGN;
  if (step != 0) {
    encoderCount += step;
    lastDirection = step > 0 ? 1 : -1;
    lastEdgeMicros = micros();
    seenValidTransition = true;
  } else if (currentAB != previousAB) {
    // Both channels changed between samples: noise or missed transition.
    invalidTransitions++;
  }
  previousAB = currentAB;
}

void setup() {
  // High-impedance taps: the existing Mega/encoder circuit owns pull-ups.
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  previousAB = readAB();
  lastEdgeMicros = micros();
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onEncoderChange, CHANGE);
  Serial.begin(115200);
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();