
// ==================================================VERSION-2============================================

/*
  Robust ESP8266 Quadrature → mm (0.1 mm resolution)
  - Keeps your original logic (encX/encY are "points")
  - Adds startup debounce/stabilization and WDT-friendly yields
  - Removes possible boot-time queue spam by seeding lastSentPoints
*/

#include <Arduino.h>

// ---- Pins (NodeMCU safe) ----
const uint8_t X_A_PIN = D1;  // GPIO5
const uint8_t X_B_PIN = D2;  // GPIO4
const uint8_t Y_A_PIN = D5;  // GPIO14
const uint8_t Y_B_PIN = D6;  // GPIO12

// ---- Queue config (power-of-two) ----
#ifndef QUEUE_ORDER
#define QUEUE_ORDER 10  // 1024 entries (~8 KB)
#endif
const size_t QUEUE_SIZE = (1UL << QUEUE_ORDER);
const size_t QUEUE_MASK = QUEUE_SIZE - 1;

// ---- Buffer struct holds mm*100 values ----
struct Point {
  int32_t x_mm100;
  int32_t y_mm100;
};

// ring buffer (volatile because ISR/loop can access)
volatile Point ringBuf[QUEUE_SIZE];
volatile uint32_t bufHead = 0;
volatile uint32_t bufTail = 0;
volatile uint32_t bufCount = 0;

// ---- Encoder raw counters (these are already "points" in your setup) ----
volatile int32_t encX = 0;
volatile int32_t encY = 0;

// ---- previous A/B states ----
volatile uint8_t prevX = 0;
volatile uint8_t prevY = 0;

// ---- recording flag ----
volatile bool recording = false;

// ---- last-sent points sentinel (points units) ----
volatile int32_t lastSentPointsX = 0x7FFFFFFF;
volatile int32_t lastSentPointsY = 0x7FFFFFFF;

// ---- mapping / thresholds ----
// 360 points = 1 mm  => POINTS_PER_MM = 360
const int32_t POINTS_PER_MM = 360;       // points per 1 mm
const int32_t POINTS_PER_HALF_MM = 36;  // 0.1 mm threshold (in points)
// conversion: mm100 = round(points * 100 / POINTS_PER_MM)

// ---- masks for fast GPIO read ----
const uint32_t X_A_MASK = digitalPinToBitMask(X_A_PIN);
const uint32_t X_B_MASK = digitalPinToBitMask(X_B_PIN);
const uint32_t Y_A_MASK = digitalPinToBitMask(Y_A_PIN);
const uint32_t Y_B_MASK = digitalPinToBitMask(Y_B_PIN);

// ---- transition table (prev<<2 | curr) -> -1/0/+1 ----
const int8_t transTable[16] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

// ---- Startup stabilization period (ms) ----
const uint32_t STARTUP_STABLE_MS = 200;  // adjust up if board bounce persists

// ---- ISR: update counters only (kept minimal) ----
ICACHE_RAM_ATTR void isrX() {
  uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t a = (in & X_A_MASK) ? 1 : 0;
  uint8_t b = (in & X_B_MASK) ? 1 : 0;
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (prevX << 2) | curr;
  int8_t d = transTable[idx];
  if (d != 0) {
    encX += d;
    prevX = curr;
  } else {
    if (curr != prevX) prevX = curr;
  }
}
ICACHE_RAM_ATTR void isrY() {
  uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
  uint8_t a = (in & Y_A_MASK) ? 1 : 0;
  uint8_t b = (in & Y_B_MASK) ? 1 : 0;
  uint8_t curr = (a << 1) | b;
  uint8_t idx = (prevY << 2) | curr;
  int8_t d = transTable[idx];
  if (d != 0) {
    encY += d;
    prevY = curr;
  } else {
    if (curr != prevY) prevY = curr;
  }
}

// ---- Push into ring buffer from main (atomic) ----
void pushPointMain(int32_t x_mm100, int32_t y_mm100) {
  noInterrupts();
  if (bufCount == QUEUE_SIZE) {
    // overwrite oldest
    bufTail = (bufTail + 1) & QUEUE_MASK;
    bufCount--;
  }
  ringBuf[bufHead].x_mm100 = x_mm100;
  ringBuf[bufHead].y_mm100 = y_mm100;
  bufHead = (bufHead + 1) & QUEUE_MASK;
  bufCount++;
  interrupts();
}

// ---- Pop from ring buffer (atomic) ----
bool popPointMain(Point &out) {
  bool ok = false;
  noInterrupts();
  if (bufCount > 0) {
    out.x_mm100 = ringBuf[bufTail].x_mm100;
    out.y_mm100 = ringBuf[bufTail].y_mm100;
    bufTail = (bufTail + 1) & QUEUE_MASK;
    bufCount--;
    ok = true;
  }
  interrupts();
  return ok;
}

uint32_t getQueueCount() {
  uint32_t c;
  noInterrupts();
  c = bufCount;
  interrupts();
  return c;
}

// ---- integer math conversion: points -> mm*100 (rounded) ----
ICACHE_RAM_ATTR int32_t points_to_mm100(int32_t points) {
  // mm100 = round(points * 100 / POINTS_PER_MM)
  int64_t prod = (int64_t)points * 100LL;
  if (prod >= 0) return (int32_t)((prod + POINTS_PER_MM / 2) / POINTS_PER_MM);
  else return (int32_t)((prod - POINTS_PER_MM / 2) / POINTS_PER_MM);
}

// ---- fast int-to-ascii for positive numbers (helper for fractional) ----
char *intToAsciiNoSign(uint32_t u, char *dest) {
  char tmp[12];
  int idx = 0;
  if (u == 0) tmp[idx++] = '0';
  while (u > 0) {
    tmp[idx++] = '0' + (u % 10);
    u /= 10;
  }
  char *p = dest;
  for (int i = idx - 1; i >= 0; --i) *p++ = tmp[i];
  return p;
}

// ---- write mm100 into buffer as "int.frac" with sign ----
char *format_mm100_into(int32_t mm100, char *p) {
  if (mm100 < 0) {
    *p++ = '-';
    mm100 = -mm100;
  }
  uint32_t ip = (uint32_t)(mm100 / 100);
  uint32_t frac = (uint32_t)(mm100 % 100);
  p = intToAsciiNoSign(ip, p);
  *p++ = '.';
  *p++ = '0' + (frac / 10);
  *p++ = '0' + (frac % 10);
  return p;
}

// ---- send Point as "X.mm,Y.mm\n" ----
void sendXY(const Point &pt) {
  char buf[32];
  char *q = buf;
  q = format_mm100_into(pt.x_mm100, q);
  *q++ = ',';
  q = format_mm100_into(pt.y_mm100, q);
  *q++ = '\n';
  Serial.write((const uint8_t *)buf, q - buf);
}

// ---- Serial command handler ----
// ---- Serial command handler ----
void handleSerialCommand(String cmd) {
  cmd.trim();
  String l = cmd;
  l.toLowerCase();

  if (l == "record") {
    // atomically sample current encoder points
    int32_t curX_points, curY_points;
    noInterrupts();
    curX_points = encX;  // encX is already in points
    curY_points = encY;
    // seed lastSentPoints so first movement is relative to this instant
    lastSentPointsX = curX_points;
    lastSentPointsY = curY_points;
    interrupts();

    // convert to mm*100 and send immediately
    Point p;
    p.x_mm100 = points_to_mm100(curX_points);
    p.y_mm100 = points_to_mm100(curY_points);
    // send immediately so host gets the instant point
    sendXY(p);

    recording = true;
    Serial.println("Recording started.");
    return;
  }

  if (l == "stop") {
    // atomically sample current encoder points
    // int32_t curX_points, curY_points;
    // noInterrupts();
    // curX_points = encX;
    // curY_points = encY;
    // interrupts();

    // convert to mm*100 and send immediately before stopping
    // Point p;
    // p.x_mm100 = points_to_mm100(curX_points);
    // p.y_mm100 = points_to_mm100(curY_points);
    // sendXY(p);

    recording = false;
    // reset sentinel so next record will reseed from the then-current position
    // noInterrupts();
    // lastSentPointsX = 0x7FFFFFFF;
    // lastSentPointsY = 0x7FFFFFFF;
    // interrupts();

    Serial.println("Recording stopped.");
    return;
  }

  if (l == "status") {
    int32_t x_points, y_points, x_mm100, y_mm100;
    int32_t x_counts, y_counts;  // same as points for your setup
    uint32_t q;
    bool rec;
    noInterrupts();
    x_counts = encX;
    y_counts = encY;
    x_points = x_counts;
    y_points = y_counts;  // enc counts are points already
    x_mm100 = points_to_mm100(x_points);
    y_mm100 = points_to_mm100(y_points);
    q = bufCount;
    rec = recording;
    interrupts();

    Serial.printf("Status: Recording=%s\n", rec ? "YES" : "NO");
    Serial.printf("Raw (points): X=%ld Y=%ld\n", (long)x_points, (long)y_points);
    Serial.printf("Position (mm): X=%ld.%02ld Y=%ld.%02ld\n",
                  (long)(x_mm100 / 100), (long)abs(x_mm100 % 100),
                  (long)(y_mm100 / 100), (long)abs(y_mm100 % 100));
    Serial.printf("Queue: %u/%u\n", q, (unsigned)QUEUE_SIZE);
    return;
  }

  Serial.println("Unknown command. Supported: record | stop | status");
}


// ---- setup ----
void setup() {
  // The Serial.begin and a small delay are OK; banner printed once per boot
  Serial.begin(115200);
  delay(50);

  // minimal banner
  Serial.println();
  Serial.println("ESP8266 encoder -> mm (0.1mm resolution)");
  Serial.printf("Queue=%u FreeHeap=%u\n", (unsigned)QUEUE_SIZE, ESP.getFreeHeap());
  Serial.println("Commands: record | stop | status");

  // setup pins
  pinMode(X_A_PIN, INPUT_PULLUP);
  pinMode(X_B_PIN, INPUT_PULLUP);
  pinMode(Y_A_PIN, INPUT_PULLUP);
  pinMode(Y_B_PIN, INPUT_PULLUP);

  // initialize previous states (sample once)
  uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
  prevX = ((in & X_A_MASK) ? 2 : 0) | ((in & X_B_MASK) ? 1 : 0);
  prevY = ((in & Y_A_MASK) ? 2 : 0) | ((in & Y_B_MASK) ? 1 : 0);

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(X_A_PIN), isrX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(X_B_PIN), isrX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_A_PIN), isrY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_B_PIN), isrY, CHANGE);

  // short stabilization: let any contact bounce settle, then seed encoders and lastSentPoints
  // This prevents pushing spurious points that are caused by contact bounce at power-up
  uint32_t t0 = millis();
  while (millis() - t0 < STARTUP_STABLE_MS) {
    delay(5);
    yield();
  }

  // sample and seed sentinel values so nothing is immediately queued on boot
  noInterrupts();
  int32_t sx = encX;
  int32_t sy = encY;
  lastSentPointsX = sx;
  lastSentPointsY = sy;
  interrupts();

  // small extra delay to be safe (optional)
  delay(5);
}

// ---- main loop: sample counters and push only when >=0.5mm moved ----
void loop() {
  // handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.length()) handleSerialCommand(cmd);
  }

  // sample encoders atomically (encX/encY are in "points")
  int32_t pointsX, pointsY;
  noInterrupts();
  pointsX = encX;
  pointsY = encY;
  interrupts();

  if (recording) {
    bool sendNow = false;
    // initialize first time on record start (if sentinel is set)
    if (lastSentPointsX == 0x7FFFFFFF) {
      noInterrupts();
      lastSentPointsX = pointsX;
      lastSentPointsY = pointsY;
      interrupts();
    }
    // if moved by threshold in either axis (points units)
    if (abs(pointsX - lastSentPointsX) >= POINTS_PER_HALF_MM || abs(pointsY - lastSentPointsY) >= POINTS_PER_HALF_MM) {
      sendNow = true;
    }

    if (sendNow) {
      noInterrupts();
      lastSentPointsX = pointsX;
      lastSentPointsY = pointsY;
      interrupts();

      int32_t x_mm100 = points_to_mm100(pointsX);
      int32_t y_mm100 = points_to_mm100(pointsY);
      pushPointMain(x_mm100, y_mm100);
    }
  } else {
    // when not recording, reset sentinel so next record start samples current
    noInterrupts();
    lastSentPointsX = 0x7FFFFFFF;
    lastSentPointsY = 0x7FFFFFFF;
    interrupts();
  }

  // drain queue and send (as fast as host can accept)
  Point p;
  while (popPointMain(p)) {
    sendXY(p);
    // keep the watchdog happy when emitting a bunch of data
    yield();
    delay(0);  // tiny cooperative pause
  }

  // small cooperative sleep
  delay(1);
}


// ==============================================================
// Debug
/* Debug helper build - paste & flash
   - Shows whether encX/encY change
   - Shows raw A/B pin states periodically
   - Use to confirm wiring / interrupts / masks
*/

// #include <Arduino.h>

// // Pins
// const uint8_t X_A_PIN = D1;  // GPIO5
// const uint8_t X_B_PIN = D2;  // GPIO4
// const uint8_t Y_A_PIN = D5;  // GPIO14
// const uint8_t Y_B_PIN = D6;  // GPIO12

// // Queue trivial defs (not used by debug prints)
// #ifndef QUEUE_ORDER
// #define QUEUE_ORDER 10
// #endif
// const size_t QUEUE_SIZE = (1UL << QUEUE_ORDER);
// struct Point { int32_t x_mm100; int32_t y_mm100; };

// // Globals from your code
// volatile int32_t encX = 0;
// volatile int32_t encY = 0;
// volatile uint8_t prevX = 0;
// volatile uint8_t prevY = 0;

// // masks (same as your code)
// const uint32_t X_A_MASK = digitalPinToBitMask(X_A_PIN);
// const uint32_t X_B_MASK = digitalPinToBitMask(X_B_PIN);
// const uint32_t Y_A_MASK = digitalPinToBitMask(Y_A_PIN);
// const uint32_t Y_B_MASK = digitalPinToBitMask(Y_B_PIN);

// // transition table
// const int8_t transTable[16] = {
//   0, 1, -1, 0,
//   -1, 0, 0, 1,
//   1, 0, 0, -1,
//   0, -1, 1, 0
// };

// // ISR
// ICACHE_RAM_ATTR void isrX() {
//   uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
//   uint8_t a = (in & X_A_MASK) ? 1 : 0;
//   uint8_t b = (in & X_B_MASK) ? 1 : 0;
//   uint8_t curr = (a << 1) | b;
//   uint8_t idx = (prevX << 2) | curr;
//   int8_t d = transTable[idx];
//   if (d != 0) {
//     encX += d;
//     prevX = curr;
//   } else {
//     if (curr != prevX) prevX = curr;
//   }
// }
// ICACHE_RAM_ATTR void isrY() {
//   uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
//   uint8_t a = (in & Y_A_MASK) ? 1 : 0;
//   uint8_t b = (in & Y_B_MASK) ? 1 : 0;
//   uint8_t curr = (a << 1) | b;
//   uint8_t idx = (prevY << 2) | curr;
//   int8_t d = transTable[idx];
//   if (d != 0) {
//     encY += d;
//     prevY = curr;
//   } else {
//     if (curr != prevY) prevY = curr;
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   delay(50);
//   Serial.println();
//   Serial.println("DEBUG: encoder test");
//   Serial.println("Wire X to D1/D2, Y to D5/D6. Swap to test.");

//   pinMode(X_A_PIN, INPUT_PULLUP);
//   pinMode(X_B_PIN, INPUT_PULLUP);
//   pinMode(Y_A_PIN, INPUT_PULLUP);
//   pinMode(Y_B_PIN, INPUT_PULLUP);

//   uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
//   prevX = ((in & X_A_MASK) ? 2 : 0) | ((in & X_B_MASK) ? 1 : 0);
//   prevY = ((in & Y_A_MASK) ? 2 : 0) | ((in & Y_B_MASK) ? 1 : 0);

//   attachInterrupt(digitalPinToInterrupt(X_A_PIN), isrX, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(X_B_PIN), isrX, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(Y_A_PIN), isrY, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(Y_B_PIN), isrY, CHANGE);

//   // small settle
//   delay(100);
// }

// int32_t lastX = 0, lastY = 0;
// uint32_t lastReport = 0;

// void loop() {
//   // show enc counts when they change
//   noInterrupts();
//   int32_t tx = encX;
//   int32_t ty = encY;
//   interrupts();
//   if (tx != lastX || ty != lastY) {
//     Serial.printf("enc: X=%ld, Y=%ld\n", (long)tx, (long)ty);
//     lastX = tx; lastY = ty;
//   }

//   // every 200ms print raw pin states (helps verify A/B toggles)
//   uint32_t now = millis();
//   if (now - lastReport >= 200) {
//     uint32_t in = GPIO_REG_READ(GPIO_IN_ADDRESS);
//     uint8_t XA = (in & X_A_MASK) ? 1 : 0;
//     uint8_t XB = (in & X_B_MASK) ? 1 : 0;
//     uint8_t YA = (in & Y_A_MASK) ? 1 : 0;
//     uint8_t YB = (in & Y_B_MASK) ? 1 : 0;
//     Serial.printf("raw: XA=%d XB=%d  YA=%d YB=%d\n", XA, XB, YA, YB);
//     lastReport = now;
//   }

//   delay(10);
// }

