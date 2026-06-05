#include <math.h>

// ── Motor Pins ──────────────────────────────────────────
const int IN1 = 3;   // L298N IN1 → controls OUT1
const int IN2 = 2;   // L298N IN2 → controls OUT2
const int IN3 = 4;   // L298N IN3 → controls OUT3
const int IN4 = 5;   // L298N IN4 → controls OUT4

// Your wiring:
// Left  motor +ve → OUT1,  −ve → OUT2
// Right motor +ve → OUT3,  −ve → OUT4
//
// So to move FORWARD:
//   Left  motor: OUT1 HIGH, OUT2 LOW  → IN1 HIGH, IN2 LOW
//   Right motor: OUT3 HIGH, OUT4 LOW  → IN3 HIGH, IN4 LOW
//
// To move BACKWARD:
//   Left  motor: OUT1 LOW, OUT2 HIGH  → IN1 LOW,  IN2 HIGH
//   Right motor: OUT3 LOW, OUT4 HIGH  → IN3 LOW,  IN4 HIGH
//
// To turn RIGHT (left fwd, right back):
//   IN1 HIGH, IN2 LOW, IN3 LOW, IN4 HIGH
//
// To turn LEFT (left back, right fwd):
//   IN1 LOW, IN2 HIGH, IN3 HIGH, IN4 LOW

// ── Sensor Pins ─────────────────────────────────────────
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;
const int PIR_PIN  = 7;

// ── Ultrasonic Settings ──────────────────────────────────
const int US_SAMPLES      = 5;
const int US_VALID_MAX    = 300;
const int OBSTACLE_THRESH = 20;

// ── PIR Human Detection ──────────────────────────────────
const int            PIR_CONFIRM_COUNT = 5;
const int            PIR_SAMPLE_DELAY  = 60;
const unsigned long  PIR_HOLD_MS       = 4000;

// ── Motion ───────────────────────────────────────────────
const float ROBOT_SPEED  = 0.50;
const int   TURN_TIME_90 = 400;
const int   BACKUP_TIME  = 350;

// ── Path Tracking ────────────────────────────────────────
const unsigned long PATH_LOG_INTERVAL = 5000;
const float         MIN_MOVE_CM       = 2.0;
const int           MAX_PATH          = 100;

// ── Odometry ─────────────────────────────────────────────
float posX    = 0.0;
float posY    = 0.0;
float heading = 1.5708;

unsigned long lastTime       = 0;
unsigned long lastPathLog    = 0;
unsigned long humanClearedAt = 0;
bool          isMoving       = false;
bool          humanWasHere   = false;

// ── Path Storage ─────────────────────────────────────────
float pathX[MAX_PATH];
float pathY[MAX_PATH];
int   pathIndex = 0;

// ── State Machine ─────────────────────────────────────────
enum RobotState { MOVING, OBSTACLE_AVOID, HUMAN_DETECTED };
RobotState state = MOVING;

// ── Function Declarations ────────────────────────────────
bool detectHuman();
int  rawUltrasonic();
int  readUltrasonicFiltered();
int  sweepSide(bool goLeft);
void moveForward();
void moveBackward();
void stopMotors();
void turnRight();
void turnLeft();
void logPath(float x, float y);
void printFullPath();
void printTelemetry(int dist, bool pir);

// ═════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN,  INPUT);

  stopMotors();
  logPath(posX, posY);

  Serial.println("========================================");
  Serial.println("         RESCUE BOT INITIALISING        ");
  Serial.println("========================================");
  Serial.println("Warming up PIR (5 sec)...");
  delay(5000);

  lastTime    = millis();
  lastPathLog = millis();

  Serial.println("Ready!");
  Serial.println("[STATE] X:v Y:v HEAD:vdeg DIST:vcm PIR:v");
  Serial.println("========================================");
}

// ═════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (isMoving) {
    float d = ROBOT_SPEED * dt;
    posX += d * cos(heading);
    posY += d * sin(heading);
  }

  int  distance = readUltrasonicFiltered();
  bool human    = detectHuman();

  printTelemetry(distance, human);

  if (now - lastPathLog >= PATH_LOG_INTERVAL) {
    lastPathLog = now;
    logPath(posX, posY);
  }

  // ── PRIORITY 1: Human Detected ───────────────────────
  if (human) {
    humanWasHere   = true;
    humanClearedAt = now;

    if (state != HUMAN_DETECTED) {
      state = HUMAN_DETECTED;
      stopMotors();
      isMoving = false;
      logPath(posX, posY);
      Serial.println(">>> !! HUMAN CONFIRMED !! Robot halted.");
      Serial.print("    Position X:"); Serial.print(posX, 2);
      Serial.print("  Y:"); Serial.println(posY, 2);
      printFullPath();
    }
    delay(50);
    return;
  }

  if (humanWasHere) {
    if (now - humanClearedAt < PIR_HOLD_MS) {
      delay(50);
      return;
    }
    humanWasHere = false;
    state = MOVING;
    Serial.println(">>> Human gone — resuming mission.");
  }

  // ── PRIORITY 2: Obstacle Avoidance ───────────────────
  if (distance > 0 && distance < OBSTACLE_THRESH) {
    state = OBSTACLE_AVOID;
    stopMotors();
    isMoving = false;

    Serial.print(">>> OBSTACLE at "); Serial.print(distance);
    Serial.println(" cm — choosing escape...");
    logPath(posX, posY);

    moveBackward();
    delay(BACKUP_TIME);
    stopMotors();
    delay(150);

    int dLeft  = sweepSide(true);
    int dRight = sweepSide(false);

    Serial.print("    Left:"); Serial.print(dLeft);
    Serial.print("cm  Right:"); Serial.print(dRight); Serial.println("cm");

    if (dLeft >= dRight) { turnLeft();  Serial.println("    => TURN LEFT");  }
    else                 { turnRight(); Serial.println("    => TURN RIGHT"); }

    state = MOVING;
    return;
  }

  // ── DEFAULT: Move Forward ─────────────────────────────
  state = MOVING;
  moveForward();
  isMoving = true;
  delay(50);
}

// ═════════════════════════════════════════════════════════
//  HUMAN DETECTION
// ═════════════════════════════════════════════════════════
bool detectHuman() {
  int highCount = 0;
  for (int i = 0; i < PIR_CONFIRM_COUNT; i++) {
    if (digitalRead(PIR_PIN) == HIGH) highCount++;
    delay(PIR_SAMPLE_DELAY);
  }
  return (highCount == PIR_CONFIRM_COUNT);
}

// ═════════════════════════════════════════════════════════
//  ULTRASONIC
// ═════════════════════════════════════════════════════════
int rawUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 38000);
  if (dur == 0) return 999;
  return (int)(dur * 0.034 / 2);
}

int readUltrasonicFiltered() {
  int readings[US_SAMPLES];
  int count = 0;
  for (int i = 0; i < US_SAMPLES; i++) {
    int v = rawUltrasonic();
    if (v > 0 && v < US_VALID_MAX) {
      readings[count++] = v;
    }
    delay(25);
  }
  if (count == 0) return 999;
  if (count == 1) return readings[0];
  for (int i = 1; i < count; i++) {
    int key = readings[i], j = i - 1;
    while (j >= 0 && readings[j] > key) {
      readings[j + 1] = readings[j]; j--;
    }
    readings[j + 1] = key;
  }
  return readings[count / 2];
}

// ═════════════════════════════════════════════════════════
//  SWEEP
// ═════════════════════════════════════════════════════════
int sweepSide(bool goLeft) {
  if (goLeft) {
    // Left back, Right fwd
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    // Left fwd, Right back
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  delay(TURN_TIME_90);
  stopMotors();
  delay(100);

  int d = readUltrasonicFiltered();

  // Rotate back to original direction
  if (goLeft) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
  delay(TURN_TIME_90);
  stopMotors();
  delay(100);

  return d;
}

// ═════════════════════════════════════════════════════════
//  MOTOR CONTROLS — matched to your wiring
//  Left  +ve→OUT1  −ve→OUT2
//  Right +ve→OUT3  −ve→OUT4
// ═════════════════════════════════════════════════════════
void moveForward() {
  // Both motors spin forward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  fwd
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right fwd
}

void moveBackward() {
  // Both motors spin backward
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left  back
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right back
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void turnRight() {
  // Left fwd + Right back = spin clockwise = turn RIGHT
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left  fwd
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // Right back
  delay(TURN_TIME_90);
  stopMotors();
  delay(100);
  heading -= 1.5708;
  if (heading < 0) heading += 6.2832;
  Serial.print(">>> Turned RIGHT | Heading: ");
  Serial.print(degrees(heading), 1); Serial.println("deg");
}

void turnLeft() {
  // Left back + Right fwd = spin counter-clockwise = turn LEFT
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  // Left  back
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right fwd
  delay(TURN_TIME_90);
  stopMotors();
  delay(100);
  heading += 1.5708;
  if (heading >= 6.2832) heading -= 6.2832;
  Serial.print(">>> Turned LEFT  | Heading: ");
  Serial.print(degrees(heading), 1); Serial.println("deg");
}

// ═════════════════════════════════════════════════════════
//  PATH LOGGING
// ═════════════════════════════════════════════════════════
void logPath(float x, float y) {
  if (pathIndex > 0) {
    float dx = x - pathX[pathIndex - 1];
    float dy = y - pathY[pathIndex - 1];
    float distCm = sqrt(dx * dx + dy * dy) * 100.0;
    if (distCm < MIN_MOVE_CM) return;
  }
  if (pathIndex < MAX_PATH) {
    pathX[pathIndex] = x;
    pathY[pathIndex] = y;
    pathIndex++;
  } else {
    for (int i = 0; i < MAX_PATH - 1; i++) {
      pathX[i] = pathX[i + 1];
      pathY[i] = pathY[i + 1];
    }
    pathX[MAX_PATH - 1] = x;
    pathY[MAX_PATH - 1] = y;
  }
  Serial.print("[PATH] #"); Serial.print(pathIndex);
  Serial.print(" X:"); Serial.print(x, 2);
  Serial.print("  Y:"); Serial.println(y, 2);
}

void printFullPath() {
  Serial.println("======= FULL PATH LOG =======");
  for (int i = 0; i < pathIndex; i++) {
    Serial.print("  #"); Serial.print(i + 1);
    Serial.print(" X:"); Serial.print(pathX[i], 2);
    Serial.print("  Y:"); Serial.println(pathY[i], 2);
  }
  Serial.print("  Total waypoints: "); Serial.println(pathIndex);
  Serial.println("=============================");
}

// ═════════════════════════════════════════════════════════
//  TELEMETRY
// ═════════════════════════════════════════════════════════
void printTelemetry(int dist, bool pir) {
  Serial.print("[");
  switch (state) {
    case MOVING:         Serial.print("MOVING   "); break;
    case OBSTACLE_AVOID: Serial.print("AVOIDING "); break;
    case HUMAN_DETECTED: Serial.print("HUMAN!!! "); break;
  }
  Serial.print("] ");
  Serial.print("X:"); Serial.print(posX, 2);
  Serial.print("  Y:"); Serial.print(posY, 2);
  Serial.print("  HEAD:"); Serial.print(degrees(heading), 1); Serial.print("deg");
  Serial.print("  DIST:");
  if (dist == 999) Serial.print("open ");
  else { Serial.print(dist); Serial.print("cm "); }
  Serial.print("  PIR:"); Serial.println(pir ? "HUMAN" : "clear");
}
