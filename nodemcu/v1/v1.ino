/*
  ============================================================
  ESP8266 Quadrature Encoder → Position Streaming (mm)
  ============================================================

  Key Properties:
  - Encoder produces FIXED points per mechanical distance
  - Resolution controls WHEN a point is emitted, not conversion
  - No hard-coded magic numbers in logic
  - Config-first, scalable, production-friendly structure
*/

#include <Arduino.h>

/* ============================================================
   1. HARDWARE PIN CONFIGURATION
   ============================================================ */

struct EncoderPins {
  uint8_t a;
  uint8_t b;
};

const EncoderPins X_AXIS = { D1, D2 };  // GPIO5, GPIO4
const EncoderPins Y_AXIS = { D5, D6 };  // GPIO14, GPIO12

/* ============================================================
   2. ENCODER & MECHANICAL CONFIGURATION
   ============================================================ */

/*
  Mechanical truth:
    360 encoder points = 10 mm = 1 cm
*/
struct EncoderScaleConfig {
  int32_t pointsPerConfiguredDistance;
  int32_t configuredDistance_mm;
};

const EncoderScaleConfig ENCODER_SCALE = {
  .pointsPerConfiguredDistance = 360,
  .configuredDistance_mm       = 10
};

// Derived (computed, NOT hard-coded)
const int32_t POINTS_PER_MM =
  ENCODER_SCALE.pointsPerConfiguredDistance /
  ENCODER_SCALE.configuredDistance_mm;

/* ============================================================
   3. SAMPLING / OUTPUT RESOLUTION CONFIGURATION
   ============================================================ */

/*
  Controls WHEN a new point is emitted.
  This is NOT unit conversion.
*/
struct SamplingConfig {
  int32_t emitEvery_mm;   // e.g. 1 = every 1 mm
};

const SamplingConfig SAMPLING = {
  .emitEvery_mm = 1
};

const int32_t POINTS_PER_EMIT_STEP =
  SAMPLING.emitEvery_mm * POINTS_PER_MM;

/* ============================================================
   4. RING BUFFER CONFIGURATION
   ============================================================ */

#ifndef QUEUE_ORDER
#define QUEUE_ORDER 10     // 2^10 = 1024 entries
#endif

const size_t QUEUE_SIZE = (1UL << QUEUE_ORDER);
const size_t QUEUE_MASK = QUEUE_SIZE - 1;

/* ============================================================
   5. DATA STRUCTURES
   ============================================================ */

struct PointMM100 {
  int32_t x;
  int32_t y;
};

volatile PointMM100 ringBuffer[QUEUE_SIZE];
volatile uint32_t bufferHead = 0;
volatile uint32_t bufferTail = 0;
volatile uint32_t bufferCount = 0;

/* ============================================================
   6. ENCODER STATE
   ============================================================ */

volatile int32_t encoderX_points = 0;
volatile int32_t encoderY_points = 0;

volatile uint8_t prevXState = 0;
volatile uint8_t prevYState = 0;

volatile bool isRecording = false;

/* Sentinel used to detect first sample after record start */
volatile int32_t lastEmittedX_points = INT32_MAX;
volatile int32_t lastEmittedY_points = INT32_MAX;

/* ============================================================
   7. FAST GPIO ACCESS
   ============================================================ */

const uint32_t X_A_MASK = digitalPinToBitMask(X_AXIS.a);
const uint32_t X_B_MASK = digitalPinToBitMask(X_AXIS.b);
const uint32_t Y_A_MASK = digitalPinToBitMask(Y_AXIS.a);
const uint32_t Y_B_MASK = digitalPinToBitMask(Y_AXIS.b);

/* ============================================================
   8. QUADRATURE TRANSITION TABLE
   ============================================================ */

const int8_t QUAD_TRANSITION[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

/* ============================================================
   9. ISR IMPLEMENTATION
   ============================================================ */

ICACHE_RAM_ATTR void handleEncoderX() {
  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t curr = ((gpio & X_A_MASK) ? 2 : 0) | ((gpio & X_B_MASK) ? 1 : 0);
  uint8_t idx = (prevXState << 2) | curr;
  int8_t delta = QUAD_TRANSITION[idx];

  if (delta != 0) encoderX_points += delta;
  prevXState = curr;
}

ICACHE_RAM_ATTR void handleEncoderY() {
  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t curr = ((gpio & Y_A_MASK) ? 2 : 0) | ((gpio & Y_B_MASK) ? 1 : 0);
  uint8_t idx = (prevYState << 2) | curr;
  int8_t delta = QUAD_TRANSITION[idx];

  if (delta != 0) encoderY_points += delta;
  prevYState = curr;
}

/* ============================================================
   10. CONVERSION UTILITIES
   ============================================================ */

inline int32_t pointsToMM100(int32_t points) {
  int64_t scaled = (int64_t)points * 100;
  return (scaled >= 0)
         ? (scaled + POINTS_PER_MM / 2) / POINTS_PER_MM
         : (scaled - POINTS_PER_MM / 2) / POINTS_PER_MM;
}

/* ============================================================
   11. RING BUFFER HELPERS
   ============================================================ */

void enqueuePoint(int32_t x_mm100, int32_t y_mm100) {
  noInterrupts();
  if (bufferCount == QUEUE_SIZE) {
    bufferTail = (bufferTail + 1) & QUEUE_MASK;
    bufferCount--;
  }
  ringBuffer[bufferHead] = { x_mm100, y_mm100 };
  bufferHead = (bufferHead + 1) & QUEUE_MASK;
  bufferCount++;
  interrupts();
}

bool dequeuePoint(PointMM100 &out) {
  noInterrupts();
  if (!bufferCount) {
    interrupts();
    return false;
  }
  out = ringBuffer[bufferTail];
  bufferTail = (bufferTail + 1) & QUEUE_MASK;
  bufferCount--;
  interrupts();
  return true;
}

/* ============================================================
   12. SERIAL OUTPUT
   ============================================================ */

void sendPoint(const PointMM100 &p) {
  Serial.printf("%ld.%02ld,%ld.%02ld\n",
    p.x / 100, abs(p.x % 100),
    p.y / 100, abs(p.y % 100)
  );
}

/* ============================================================
   13. COMMAND PROCESSOR (SCALABLE)
   ============================================================ */

void processCommand(const String &rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();

  // ---- Backward-compatible aliases ----
  if (cmd == "record") cmd = "record:start";
  if (cmd == "stop")   cmd = "record:stop";

  // ---- Command handling ----
  if (cmd == "record:start") {
    noInterrupts();
    lastEmittedX_points = encoderX_points;
    lastEmittedY_points = encoderY_points;
    interrupts();

    isRecording = true;
    Serial.println("OK RECORDING STARTED");
    return;
  }

  if (cmd == "record:stop") {
    isRecording = false;
    Serial.println("OK RECORDING STOPPED");
    return;
  }

  if (cmd == "status") {
    noInterrupts();
    int32_t x = encoderX_points;
    int32_t y = encoderY_points;
    interrupts();

    Serial.printf(
      "STATUS | raw=(%ld,%ld) | mm=(%ld.%02ld,%ld.%02ld) | q=%u | res=%dmm\n",
      x, y,
      pointsToMM100(x) / 100, abs(pointsToMM100(x) % 100),
      pointsToMM100(y) / 100, abs(pointsToMM100(y) % 100),
      bufferCount,
      SAMPLING.emitEvery_mm
    );
    return;
  }

  Serial.println("ERR UNKNOWN COMMAND");
}

/* ============================================================
   14. SETUP
   ============================================================ */

void setup() {
  Serial.begin(115200);
  delay(50);

  Serial.println("\nESP8266 Encoder Streaming Ready");
  Serial.printf("Scale: %d points = %d mm\n",
    ENCODER_SCALE.pointsPerConfiguredDistance,
    ENCODER_SCALE.configuredDistance_mm
  );
  Serial.printf("Resolution: %d mm\n", SAMPLING.emitEvery_mm);
  Serial.println("Commands: record:start | record:stop | status");

  pinMode(X_AXIS.a, INPUT_PULLUP);
  pinMode(X_AXIS.b, INPUT_PULLUP);
  pinMode(Y_AXIS.a, INPUT_PULLUP);
  pinMode(Y_AXIS.b, INPUT_PULLUP);

  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  prevXState = ((gpio & X_A_MASK) ? 2 : 0) | ((gpio & X_B_MASK) ? 1 : 0);
  prevYState = ((gpio & Y_A_MASK) ? 2 : 0) | ((gpio & Y_B_MASK) ? 1 : 0);

  attachInterrupt(digitalPinToInterrupt(X_AXIS.a), handleEncoderX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_AXIS.b), handleEncoderX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_AXIS.a), handleEncoderY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_AXIS.b), handleEncoderY, CHANGE);
}

/* ============================================================
   15. MAIN LOOP
   ============================================================ */

void loop() {
  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }

  if (isRecording) {
    int32_t x, y;
    noInterrupts();
    x = encoderX_points;
    y = encoderY_points;
    interrupts();

    if (abs(x - lastEmittedX_points) >= POINTS_PER_EMIT_STEP ||
        abs(y - lastEmittedY_points) >= POINTS_PER_EMIT_STEP) {

      lastEmittedX_points = x;
      lastEmittedY_points = y;

      enqueuePoint(pointsToMM100(x), pointsToMM100(y));
    }
  }

  PointMM100 p;
  while (dequeuePoint(p)) {
    sendPoint(p);
    yield();
  }

  delay(1);
}
