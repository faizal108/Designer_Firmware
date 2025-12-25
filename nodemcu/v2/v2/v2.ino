/*
  ============================================================
  ESP8266 Quadrature Encoder → Position Streaming
  DUAL OUTPUT:
   - Serial : ASCII (debug / control)
   - WiFi   : Binary framed (high speed)
  ============================================================
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
   2. ENCODER SCALE (UNCHANGED)
   ============================================================ */

typedef struct {
  int32_t pointsPerConfiguredDistance;
  int32_t configuredDistance_mm;
} EncoderScaleConfig;

const EncoderScaleConfig ENCODER_SCALE = { 360, 10 };

const int32_t POINTS_PER_MM =
  ENCODER_SCALE.pointsPerConfiguredDistance / ENCODER_SCALE.configuredDistance_mm;

/* ============================================================
   3. RUNTIME RESOLUTION
   ============================================================ */

volatile int32_t emitStep_points = 0;
volatile bool emitEveryPoint = false;

/* ============================================================
   4. LOCK-FREE RING BUFFER
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
   6. FAST GPIO
   ============================================================ */

const uint32_t X_A_MASK = digitalPinToBitMask(X_AXIS.a);
const uint32_t X_B_MASK = digitalPinToBitMask(X_AXIS.b);
const uint32_t Y_A_MASK = digitalPinToBitMask(Y_AXIS.a);
const uint32_t Y_B_MASK = digitalPinToBitMask(Y_AXIS.b);

/* ============================================================
   7. QUADRATURE TABLE
   ============================================================ */

const int8_t QUAD_TRANSITION[16] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

/* ============================================================
   8. ISR
   ============================================================ */

ICACHE_RAM_ATTR void handleEncoderX() {
  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t curr = ((gpio & X_A_MASK) ? 2 : 0) | ((gpio & X_B_MASK) ? 1 : 0);
  int8_t d = QUAD_TRANSITION[(prevXState << 2) | curr];
  if (d) encoderX_points += d;
  prevXState = curr;
}

ICACHE_RAM_ATTR void handleEncoderY() {
  uint32_t gpio = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t curr = ((gpio & Y_A_MASK) ? 2 : 0) | ((gpio & Y_B_MASK) ? 1 : 0);
  int8_t d = QUAD_TRANSITION[(prevYState << 2) | curr];
  if (d) encoderY_points += d;
  prevYState = curr;
}

/* ============================================================
   9. CONVERSION
   ============================================================ */

static inline int32_t pointsToMM100(int32_t points) {
  int64_t v = (int64_t)points * 100;
  return (v >= 0) ? (v + POINTS_PER_MM / 2) / POINTS_PER_MM
                  : (v - POINTS_PER_MM / 2) / POINTS_PER_MM;
}

/* ============================================================
   10. QUEUE OPS
   ============================================================ */

void enqueuePoint(int32_t x, int32_t y) {
  uint32_t next = RING_NEXT(bufferHead);
  if (next == bufferTail) {
    bufferTail = RING_NEXT(bufferTail);
    droppedPoints++;
  }
  ringBuffer[bufferHead] = { x, y };
  bufferHead = next;
}

bool dequeuePoint(PointMM100 *out) {
  if (bufferHead == bufferTail) return false;
  *out = ringBuffer[bufferTail];
  bufferTail = RING_NEXT(bufferTail);
  return true;
}

/* ============================================================
   11. WIFI STATE
   ============================================================ */

WiFiServer wifiServer(9000);
WiFiClient wifiClient;

char wifiSSID[32] = { 0 };
char wifiPASS[32] = { 0 };
bool wifiEnabled = false;

/* ============================================================
   12. ASCII SERIAL OUTPUT
   ============================================================ */

void sendAsciiSerial(const PointMM100 *p) {
  Serial.printf(
    "%ld.%02ld,%ld.%02ld\n",
    p->x / 100, abs(p->x % 100),
    p->y / 100, abs(p->y % 100));
}

/* ============================================================
   13. BINARY WIFI OUTPUT
   ============================================================ */

uint8_t crc8(const uint8_t *buf, uint8_t len) {
  uint8_t c = 0;
  while (len--) c ^= *buf++;
  return c;
}

void sendBinaryWifi(const PointMM100 *p) {
  if (!wifiClient || !wifiClient.connected()) return;

  uint8_t frame[13];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = 0x01;
  frame[3] = 8;

  memcpy(&frame[4], &p->x, 4);
  memcpy(&frame[8], &p->y, 4);

  frame[12] = crc8(frame, 12);
  wifiClient.write(frame, sizeof(frame));
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

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("ERR WIFI CONNECT FAILED");
    return;
  }

  wifiServer.begin();
  wifiEnabled = true;
  Serial.print("OK WIFI IP ");
  Serial.println(WiFi.localIP());
}

/* ============================================================
   15. RESOLUTION
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
   16. COMMANDS (SERIAL ONLY)
   ============================================================ */

void processCommand(String cmd) {
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
    float mm = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    updateResolutionFromMM(mm);
    Serial.printf("OK RESOLUTION %.3f mm\n", mm);
    return;
  }

  if (cmd.startsWith("wifi:config")) {
    int a = cmd.indexOf(' ');
    int b = cmd.indexOf(' ', a + 1);
    cmd.substring(a + 1, b).toCharArray(wifiSSID, 32);
    cmd.substring(b + 1).toCharArray(wifiPASS, 32);
    Serial.println("OK WIFI CONFIG SAVED");
    return;
  }

  if (cmd == "wifi:on") {
    startWifi();
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

  Serial.println("ESP8266 Encoder Ready (Dual Output)");
}

/* ============================================================
   18. LOOP
   ============================================================ */

void loop() {
  if (Serial.available())
    processCommand(Serial.readStringUntil('\n'));

  if (wifiEnabled && !wifiClient) {
    wifiClient = wifiServer.available();
    if (wifiClient) Serial.println("OK WIFI CLIENT CONNECTED");
  }

  if (isRecording) {
    int32_t x = encoderX_points;
    int32_t y = encoderY_points;

    bool emit = emitEveryPoint ? (x != lastEmittedX_points || y != lastEmittedY_points) : (abs(x - lastEmittedX_points) >= emitStep_points || abs(y - lastEmittedY_points) >= emitStep_points);

    if (emit) {
      lastEmittedX_points = x;
      lastEmittedY_points = y;
      enqueuePoint(pointsToMM100(x), pointsToMM100(y));
    }
  }

  PointMM100 p;
  uint8_t burst = wifiEnabled ? 4 : 32;

  while (burst-- && dequeuePoint(&p)) {
    sendAsciiSerial(&p);
    sendBinaryWifi(&p);
    yield();
  }

  delay(1);
}
