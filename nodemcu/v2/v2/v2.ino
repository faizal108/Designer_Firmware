/*
  ============================================================
  ESP8266 Quadrature Encoder → Position Streaming (mm)
  SERIAL + WIFI OUTPUT (QUEUE SAFE)
  ============================================================

  NOTE:
  - Encoder math untouched
  - ISR untouched
  - Queue untouched
  - Only output transport extended
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

/* ============================================================
   1. HARDWARE PIN CONFIGURATION
   ============================================================ */

typedef struct {
  uint8_t a;
  uint8_t b;
} EncoderPins;

const EncoderPins X_AXIS = { D1, D2 };
const EncoderPins Y_AXIS = { D5, D6 };

/* ============================================================
   2. ENCODER & MECHANICAL CONFIGURATION
   ============================================================ */

typedef struct {
  int32_t pointsPerConfiguredDistance;
  int32_t configuredDistance_mm;
} EncoderScaleConfig;

const EncoderScaleConfig ENCODER_SCALE = {
  360,
  10
};

const int32_t POINTS_PER_MM =
  ENCODER_SCALE.pointsPerConfiguredDistance / ENCODER_SCALE.configuredDistance_mm;

/* ============================================================
   3. RUNTIME RESOLUTION CONFIGURATION
   ============================================================ */

volatile int32_t emitStep_points = 0;
volatile bool emitEveryPoint = false;

/* ============================================================
   4. RING BUFFER CONFIGURATION (LOCK-FREE)
   ============================================================ */

#ifndef QUEUE_ORDER
#define QUEUE_ORDER 10
#endif

#define QUEUE_SIZE (1UL << QUEUE_ORDER)
#define QUEUE_MASK (QUEUE_SIZE - 1)
#define RING_NEXT(i) (((i) + 1) & QUEUE_MASK)

typedef struct {
  int32_t x;
  int32_t y;
} PointMM100;

volatile PointMM100 ringBuffer[QUEUE_SIZE];
volatile uint32_t bufferHead = 0;
volatile uint32_t bufferTail = 0;
volatile uint32_t droppedPoints = 0;

static inline uint32_t ringCount(void) {
  return (bufferHead - bufferTail) & QUEUE_MASK;
}

/* ============================================================
   5. ENCODER STATE
   ============================================================ */

volatile int32_t encoderX_points = 0;
volatile int32_t encoderY_points = 0;

volatile uint8_t prevXState = 0;
volatile uint8_t prevYState = 0;

volatile bool isRecording = false;
volatile int32_t lastEmittedX_points = INT32_MAX;
volatile int32_t lastEmittedY_points = INT32_MAX;

/* ============================================================
   6. FAST GPIO ACCESS
   ============================================================ */

const uint32_t X_A_MASK = digitalPinToBitMask(X_AXIS.a);
const uint32_t X_B_MASK = digitalPinToBitMask(X_AXIS.b);
const uint32_t Y_A_MASK = digitalPinToBitMask(Y_AXIS.a);
const uint32_t Y_B_MASK = digitalPinToBitMask(Y_AXIS.b);

/* ============================================================
   7. QUADRATURE TRANSITION TABLE
   ============================================================ */

const int8_t QUAD_TRANSITION[16] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

/* ============================================================
   8. ISR IMPLEMENTATION
   ============================================================ */

ICACHE_RAM_ATTR void handleEncoderX() {
  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t curr = ((gpio & X_A_MASK) ? 2 : 0) | ((gpio & X_B_MASK) ? 1 : 0);
  uint8_t idx = (prevXState << 2) | curr;
  int8_t d = QUAD_TRANSITION[idx];
  if (d) encoderX_points += d;
  prevXState = curr;
}

ICACHE_RAM_ATTR void handleEncoderY() {
  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t curr = ((gpio & Y_A_MASK) ? 2 : 0) | ((gpio & Y_B_MASK) ? 1 : 0);
  uint8_t idx = (prevYState << 2) | curr;
  int8_t d = QUAD_TRANSITION[idx];
  if (d) encoderY_points += d;
  prevYState = curr;
}

/* ============================================================
   9. CONVERSION UTILITIES
   ============================================================ */

static inline int32_t pointsToMM100(int32_t points) {
  int64_t v = (int64_t)points * 100;
  return (v >= 0) ? (v + POINTS_PER_MM / 2) / POINTS_PER_MM
                  : (v - POINTS_PER_MM / 2) / POINTS_PER_MM;
}

/* ============================================================
   10. LOCK-FREE RING BUFFER
   ============================================================ */

void enqueuePoint(int32_t x_mm100, int32_t y_mm100) {
  uint32_t next = RING_NEXT(bufferHead);

  if (next == bufferTail) {
    bufferTail = RING_NEXT(bufferTail);
    droppedPoints++;
  }

  ringBuffer[bufferHead].x = x_mm100;
  ringBuffer[bufferHead].y = y_mm100;
  bufferHead = next;
}

bool dequeuePoint(PointMM100 *out) {
  if (bufferHead == bufferTail) return false;

  out->x = ringBuffer[bufferTail].x;
  out->y = ringBuffer[bufferTail].y;
  bufferTail = RING_NEXT(bufferTail);
  return true;
}

/* ============================================================
   11. OUTPUT MODES
   ============================================================ */

enum OutputMode {
  OUTPUT_SERIAL,
  OUTPUT_WIFI
};

volatile OutputMode outputMode = OUTPUT_SERIAL;

/* ============================================================
   12. WIFI STATE
   ============================================================ */

WiFiServer wifiServer(9000);
WiFiClient wifiClient;

char wifiSSID[32] = { 0 };
char wifiPASS[32] = { 0 };
bool wifiEnabled = false;

/* ============================================================
   13. OUTPUT IMPLEMENTATION
   ============================================================ */

void sendPointSerial(const PointMM100 *p) {
  Serial.printf(
    "%ld.%02ld,%ld.%02ld\n",
    p->x / 100, abs(p->x % 100),
    p->y / 100, abs(p->y % 100));
}

void sendPointWifi(const PointMM100 *p) {
  if (!wifiClient || !wifiClient.connected()) return;

  wifiClient.printf(
    "%ld.%02ld,%ld.%02ld\n",
    p->x / 100, abs(p->x % 100),
    p->y / 100, abs(p->y % 100));
}

void sendPoint(const PointMM100 *p) {
  if (outputMode == OUTPUT_WIFI) {
    sendPointWifi(p);
  } else {
    sendPointSerial(p);
  }
}

/* ============================================================
   14. WIFI CONTROL
   ============================================================ */

void startWifi() {
  if (!wifiSSID[0]) {
    Serial.println("ERR WIFI NOT CONFIGURED");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPASS);

  Serial.print("WIFI CONNECTING");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR WIFI CONNECT FAILED");
    return;
  }

  wifiServer.begin();
  wifiEnabled = true;

  Serial.print("OK WIFI IP ");
  Serial.println(WiFi.localIP());
}

void stopWifi() {
  if (wifiClient) wifiClient.stop();
  wifiServer.stop();
  WiFi.disconnect(true);
  wifiEnabled = false;
  Serial.println("OK WIFI STOPPED");
}

/* ============================================================
   15. RESOLUTION HANDLING
   ============================================================ */

void updateResolutionFromMM(float mm) {
  if (mm <= 0.0f) {
    emitEveryPoint = true;
    emitStep_points = 1;
  } else {
    emitEveryPoint = false;
    emitStep_points = (int32_t)(mm * POINTS_PER_MM + 0.5f);
    if (emitStep_points < 1) emitStep_points = 1;
  }
}

/* ============================================================
   16. COMMAND PROCESSOR
   ============================================================ */

void processCommand(const String &rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "record") cmd = "record:start";
  if (cmd == "stop") cmd = "record:stop";

  if (cmd == "record:start") {
    lastEmittedX_points = encoderX_points;
    lastEmittedY_points = encoderY_points;
    isRecording = true;
    Serial.println("OK RECORDING STARTED");
    return;
  }

  if (cmd == "record:stop") {
    isRecording = false;
    Serial.println("OK RECORDING STOPPED");
    return;
  }

  if (cmd.startsWith("config:resolution")) {
    int sp = cmd.indexOf(' ');
    if (sp < 0) {
      Serial.println("ERR MISSING VALUE");
      return;
    }
    float mm = cmd.substring(sp + 1).toFloat();
    updateResolutionFromMM(mm);
    Serial.printf("OK RESOLUTION %.3f mm\n", mm);
    return;
  }

  if (cmd == "point") {
    int32_t x = encoderX_points;
    int32_t y = encoderY_points;
    Serial.printf("POSITION: X=%ld.%02ld,Y=%ld.%02ld\n",
                  pointsToMM100(x) / 100, abs(pointsToMM100(x) % 100),
                  pointsToMM100(y) / 100, abs(pointsToMM100(y) % 100));
    return;
  }

  if (cmd == "status") {
    int32_t x = encoderX_points;
    int32_t y = encoderY_points;
    Serial.printf(
      "STATUS | raw=(%ld,%ld) | mm=(%ld.%02ld,%ld.%02ld) | q=%u | drop=%u | step=%dpt\n",
      x, y,
      pointsToMM100(x) / 100, abs(pointsToMM100(x) % 100),
      pointsToMM100(y) / 100, abs(pointsToMM100(y) % 100),
      ringCount(),
      droppedPoints,
      emitStep_points);
    return;
  }

  if (cmd.startsWith("wifi:config")) {
    int a = cmd.indexOf(' ');
    int b = cmd.indexOf(' ', a + 1);
    if (a < 0 || b < 0) {
      Serial.println("ERR WIFI CONFIG FORMAT");
      return;
    }
    cmd.substring(a + 1, b).toCharArray(wifiSSID, sizeof(wifiSSID));
    cmd.substring(b + 1).toCharArray(wifiPASS, sizeof(wifiPASS));
    Serial.println("OK WIFI CONFIG SAVED");
    return;
  }

  if (cmd == "wifi:on") {
    startWifi();
    return;
  }

  if (cmd == "wifi:off") {
    stopWifi();
    return;
  }

  if (cmd == "output:serial") {
    outputMode = OUTPUT_SERIAL;
    Serial.println("OK OUTPUT SERIAL");
    return;
  }

  if (cmd == "output:wifi") {
    if (!wifiEnabled) {
      Serial.println("ERR WIFI NOT STARTED");
      return;
    }
    outputMode = OUTPUT_WIFI;
    Serial.println("OK OUTPUT WIFI");
    return;
  }

  Serial.println("ERR UNKNOWN COMMAND");
}

/* ============================================================
   17. SETUP
   ============================================================ */

void setup() {
  Serial.begin(115200);
  delay(50);

  updateResolutionFromMM(1.0f);

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

  Serial.println("ESP8266 Encoder Ready");
}

/* ============================================================
   18. MAIN LOOP
   ============================================================ */

void loop() {
  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }

  if (wifiEnabled && !wifiClient) {
    wifiClient = wifiServer.available();
    if (wifiClient) {
      Serial.println("OK WIFI CLIENT CONNECTED");
    }
  }

  if (isRecording) {
    int32_t x = encoderX_points;
    int32_t y = encoderY_points;

    bool emit =
      emitEveryPoint
        ? (x != lastEmittedX_points || y != lastEmittedY_points)
        : (abs(x - lastEmittedX_points) >= emitStep_points || abs(y - lastEmittedY_points) >= emitStep_points);

    if (emit) {
      lastEmittedX_points = x;
      lastEmittedY_points = y;
      enqueuePoint(pointsToMM100(x), pointsToMM100(y));
    }
  }

  PointMM100 p;
  uint8_t burst = (outputMode == OUTPUT_WIFI) ? 4 : 32;

  while (burst-- && dequeuePoint(&p)) {
    sendPoint(&p);
    yield();
  }

  delay(1);
}
