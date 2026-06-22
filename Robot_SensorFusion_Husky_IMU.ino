#include <Wire.h>
#include "HUSKYLENS.h"
#include <MPU6050.h>
#include <Servo.h>

HUSKYLENS huskylens;
MPU6050 mpu;
Servo myservo;

// =======================
// Motor driver pins (L298N)
// =======================
#define ENA 6
#define ENB 5
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

int carSpeed = 200;
const int TARGET_ID = 1;

// =======================
// Ultrasonic + Servo
// =======================
const int Trig = 12;
const int Echo = 13;
const int SERVO_PIN = 3;

int centerAngle = 90;
int rightAngle  = 30;
int leftAngle   = 150;
const int OBSTACLE_DIST = 15;   // Stop if an obstacle is closer than 15 cm

// =======================
// IR Distance Sensor (Sharp) – Analog
// =======================
const int IR_PIN = A0;          // Connect IR sensor output to A0
const int IR_MIN_CM = 10;       // Valid distance range for IR (tune if needed)
const int IR_MAX_CM = 80;

// =======================
// IMU settings
// =======================
bool imuOK = false;
const int tiltThreshold = 14200; // Lower threshold = more sensitive tilt detection

// =======================
// HuskyLens flag
// =======================
bool huskyOK = false;

// =======================
// 🔻🔻🔻 Maze + A* (Assumption Mode)
// =======================
bool mazeMode = true;                 // 🔻🔻🔻 if true → use A* maze navigation

const int MAZE_ROWS = 5;              // 🔻🔻🔻
const int MAZE_COLS = 5;              // 🔻🔻🔻

// 0 = free cell, 1 = wall (assumed maze layout) 🔻🔻🔻
int maze[MAZE_ROWS][MAZE_COLS] = {    // 🔻🔻🔻
  {0, 0, 0, 1, 0},
  {1, 1, 0, 1, 0},
  {0, 0, 0, 0, 0},
  {0, 1, 1, 1, 1},
  {0, 0, 0, 0, 0}
};

// Start & Goal cells (row, col) 🔻🔻🔻
int startRow = 0;                      // 🔻🔻🔻
int startCol = 0;                      // 🔻🔻🔻
int goalRow  = 4;                      // 🔻🔻🔻
int goalCol  = 4;                      // 🔻🔻🔻

// Robot heading in logical grid 🔻🔻🔻
enum Direction { DIR_UP, DIR_RIGHT, DIR_DOWN, DIR_LEFT };  // 🔻🔻🔻
Direction heading   = DIR_UP;                              // 🔻🔻🔻
int logicalRow      = startRow;                            // 🔻🔻🔻
int logicalCol      = startCol;                            // 🔻🔻🔻

struct Step {                                              // 🔻🔻🔻
  int row;
  int col;
};

const int MAX_NODES = MAZE_ROWS * MAZE_COLS;               // 🔻🔻🔻
Step path[MAX_NODES];                                      // 🔻🔻🔻
int pathLength = 0;                                        // 🔻🔻🔻
int pathIndex  = 0;                                        // 🔻🔻🔻

struct Node {                                              // 🔻🔻🔻
  int r, c;
  int g, h, f;
  int parent;
  bool open;
  bool closed;
};

Node nodes[MAX_NODES];                                     // 🔻🔻🔻

int coordToIndex(int r, int c) {                           // 🔻🔻🔻
  return r * MAZE_COLS + c;
}

int heuristic(int r1, int c1, int r2, int c2) {            // 🔻🔻🔻
  return 10 * (abs(r1 - r2) + abs(c1 - c2));
}

void initNodes() {                                         // 🔻🔻🔻
  for (int r = 0; r < MAZE_ROWS; r++) {
    for (int c = 0; c < MAZE_COLS; c++) {
      int idx = coordToIndex(r, c);
      nodes[idx].r = r;
      nodes[idx].c = c;
      nodes[idx].g = 0;
      nodes[idx].h = 0;
      nodes[idx].f = 0;
      nodes[idx].parent = -1;
      nodes[idx].open = false;
      nodes[idx].closed = false;
    }
  }
}

bool isWalkable(int r, int c) {                            // 🔻🔻🔻
  if (r < 0 || r >= MAZE_ROWS || c < 0 || c >= MAZE_COLS) return false;
  return (maze[r][c] == 0);
}

void reconstructPath(int goalIndex) {                      // 🔻🔻🔻
  pathLength = 0;
  int idx = goalIndex;
  while (idx != -1 && pathLength < MAX_NODES) {
    path[pathLength].row = nodes[idx].r;
    path[pathLength].col = nodes[idx].c;
    pathLength++;
    idx = nodes[idx].parent;
  }
  // reverse
  for (int i = 0; i < pathLength / 2; i++) {
    Step tmp = path[i];
    path[i] = path[pathLength - 1 - i];
    path[pathLength - 1 - i] = tmp;
  }

  Serial.println(F("🔻🔻🔻 A* PATH (row,col):"));
  for (int i = 0; i < pathLength; i++) {
    Serial.print(F(" -> ("));
    Serial.print(path[i].row);
    Serial.print(F(","));
    Serial.print(path[i].col);
    Serial.println(F(")"));
  }
}

bool runAStar(int sr, int sc, int gr, int gc) {            // 🔻🔻🔻
  Serial.println(F("🔻🔻🔻 Running A* on maze..."));
  initNodes();

  int startIndex = coordToIndex(sr, sc);
  int goalIndex  = coordToIndex(gr, gc);

  nodes[startIndex].g = 0;
  nodes[startIndex].h = heuristic(sr, sc, gr, gc);
  nodes[startIndex].f = nodes[startIndex].g + nodes[startIndex].h;
  nodes[startIndex].open = true;

  while (true) {
    int currentIndex = -1;
    int currentF = 999999;

    for (int i = 0; i < MAX_NODES; i++) {
      if (nodes[i].open && !nodes[i].closed && nodes[i].f < currentF) {
        currentF = nodes[i].f;
        currentIndex = i;
      }
    }

    if (currentIndex == -1) {
      Serial.println(F("🔻🔻🔻 A* failed: no path"));
      return false;
    }

    if (currentIndex == goalIndex) {
      nodes[currentIndex].closed = true;
      reconstructPath(goalIndex);
      return true;
    }

    nodes[currentIndex].open = false;
    nodes[currentIndex].closed = true;

    int cr = nodes[currentIndex].r;
    int cc = nodes[currentIndex].c;

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = { 0, 1, 0,-1};

    for (int k = 0; k < 4; k++) {
      int nr = cr + dr[k];
      int nc = cc + dc[k];

      if (!isWalkable(nr, nc)) continue;

      int nIdx = coordToIndex(nr, nc);
      if (nodes[nIdx].closed) continue;

      int newG = nodes[currentIndex].g + 10;

      if (!nodes[nIdx].open || newG < nodes[nIdx].g) {
        nodes[nIdx].g = newG;
        nodes[nIdx].h = heuristic(nr, nc, gr, gc);
        nodes[nIdx].f = nodes[nIdx].g + nodes[nIdx].h;
        nodes[nIdx].parent = currentIndex;
        nodes[nIdx].open = true;
      }
    }
  }
}

void rotateToDirection(Direction desired) {                // 🔻🔻🔻
  if (heading == desired) return;

  int diff = (int)desired - (int)heading;
  if (diff == 1 || diff == -3) {
    rightCar();
    delay(400);
    stopCar();
  } else if (diff == -1 || diff == 3) {
    leftCar();
    delay(400);
    stopCar();
  } else {
    rightCar();
    delay(400);
    stopCar();
    rightCar();
    delay(400);
    stopCar();
  }
  heading = desired;
}

void navigateMazeStep() {                                 // 🔻🔻🔻
  if (pathLength == 0) {
    Serial.println(F("🔻🔻🔻 No A* path available."));
    stopCar();
    delay(500);
    return;
  }

  if (pathIndex >= pathLength - 1) {
    Serial.println(F("🔻🔻🔻 Goal reached in maze!"));
    stopCar();
    delay(1000);
    return;
  }

  Step current = {logicalRow, logicalCol};
  Step next    = path[pathIndex + 1];

  int dr = next.row - current.row;
  int dc = next.col - current.col;

  Direction desired = heading;
  if (dr == -1 && dc == 0)      desired = DIR_UP;
  else if (dr == 1 && dc == 0)  desired = DIR_DOWN;
  else if (dr == 0 && dc == 1)  desired = DIR_RIGHT;
  else if (dr == 0 && dc == -1) desired = DIR_LEFT;

  rotateToDirection(desired);

  long dFront = getFusedFrontDistance();   // استخدام الفيوجن كـ safety 🔻🔻🔻
  if (dFront != -1 && dFront <= OBSTACLE_DIST) {
    Serial.println(F("🔻🔻🔻 Real obstacle in front, cannot follow A* step."));
    stopCar();
    delay(300);
    return;
  }

  Serial.print(F("🔻🔻🔻 Moving in maze from ("));
  Serial.print(current.row);
  Serial.print(F(","));
  Serial.print(current.col);
  Serial.print(F(") to ("));
  Serial.print(next.row);
  Serial.print(F(","));
  Serial.print(next.col);
  Serial.println(F(")"));

  forwardCar();
  delay(500); // تقريبًا خطوة واحدة في المتاهة 🔻🔻🔻
  stopCar();

  logicalRow = next.row;
  logicalCol = next.col;
  pathIndex++;
}

// =======================================================
// Motor control functions
// =======================================================
void forwardCar() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println(F("ACTION: FORWARD"));
}

void leftCar() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println(F("ACTION: LEFT"));
}

void rightCar() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("ACTION: RIGHT"));
}

void stopCar() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println(F("ACTION: STOP"));
}

// =======================================================
// Ultrasonic distance measurement (HC-SR04)
// =======================================================
long getDistanceUltrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  long duration = pulseIn(Echo, HIGH, 30000);
  if (duration == 0) return -1;

  return duration / 58;
}

// Reads ultrasonic distance at a given servo angle
long getDistanceAtAngle(int angle) {
  myservo.write(angle);
  delay(250); 
  return getDistanceUltrasonic();
}

// =======================================================
// IR distance measurement (Sharp analog sensor)
// =======================================================
long getDistanceIR() {
  int raw = analogRead(IR_PIN);

  if (raw < 50) {
    return -1;
  }

  float distance = 4800.0 / (raw - 20.0);
  long d = (long)distance;

  if (d < IR_MIN_CM || d > IR_MAX_CM) {
    return -1;
  }

  return d;
}

// =======================================================
// 🔷🔷 Sensor Fusion: combine Ultrasonic + IR into one distance
// =======================================================
long fuseDistance(long dUltra, long dIR) {
  if (dUltra < 0 && dIR < 0) {
    return -1;
  }

  if (dUltra < 0) return dIR;
  if (dIR < 0)   return dUltra;

  long diff = labs(dUltra - dIR);

  if (diff <= 5) {
    return (dUltra + dIR) / 2;
  }

  if (dIR <= 25) {
    return dIR;
  }

  return dUltra;
}

// =======================================================
// 🔷🔷 Get fused FRONT distance (Ultrasonic + IR)
// =======================================================
long getFusedFrontDistance() {
  myservo.write(centerAngle);
  delay(250);

  long dU  = getDistanceUltrasonic();
  long dIR = getDistanceIR();

  Serial.print(F("[ULTRA FRONT] "));
  if (dU < 0) Serial.println(F("Invalid"));
  else {
    Serial.print(dU);
    Serial.println(F(" cm"));
  }

  Serial.print(F("[IR    FRONT] "));
  if (dIR < 0) Serial.println(F("Invalid"));
  else {
    Serial.print(dIR);
    Serial.println(F(" cm"));
  }

  long dFused = fuseDistance(dU, dIR);

  Serial.print(F("[FUSED FRONT] "));
  if (dFused < 0) Serial.println(F("No valid distance"));
  else {
    Serial.print(dFused);
    Serial.println(F(" cm"));
  }

  return dFused;
}

// =======================================================
// Setup
// =======================================================
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopCar();

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  myservo.attach(SERVO_PIN);
  myservo.write(centerAngle);

  pinMode(IR_PIN, INPUT);

  Wire.begin();

  if (!huskylens.begin(Wire)) {
    Serial.println(F("⚠ HuskyLens NOT CONNECTED, ignoring camera tracking."));
    huskyOK = false;
  } else {
    Serial.println(F("✅ HuskyLens Ready!"));
    huskyOK = true;
  }

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("✅ MPU6050 Connected!"));
    imuOK = true;
  } else {
    Serial.println(F("⚠ MPU6050 FAILED, tilt detection disabled."));
    imuOK = false;
  }

  // 🔻🔻🔻 Run A* once at startup to compute maze path
  if (mazeMode) {
    bool ok = runAStar(startRow, startCol, goalRow, goalCol);
    if (!ok) {
      Serial.println(F("🔻🔻🔻 WARNING: No path found in maze."));
    }
  }
}

// =======================================================
// Main loop
// =======================================================
void loop() {

  // (0) IMU tilt detection → Highest priority
  if (imuOK) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print(F("az = "));
    Serial.print(az);

    if (abs(az) < tiltThreshold) {
      Serial.println(F("  -> TILT DETECTED! STOP"));
      stopCar();
      delay(200);
      return;
    } else {
      Serial.println(F("  -> SAFE (Robot is level)"));
    }
  }

  // (1) Maze Mode with A* assumption 🔻🔻🔻
  if (mazeMode) {                 // 🔻🔻🔻
    navigateMazeStep();           // 🔻🔻🔻
    delay(150);
    return;
  }

  // (2) Obstacle detection with SENSOR FUSION
  long dCenter = getFusedFrontDistance();

  if (dCenter != -1 && dCenter <= OBSTACLE_DIST) {
    Serial.println(F("🚧 Obstacle detected → scanning sides with ultrasonic..."));
    stopCar();

    long dRight = getDistanceAtAngle(rightAngle);
    Serial.print(F("[RIGHT ULTRA] "));
    if (dRight < 0) Serial.println(F("No echo"));
    else { 
      Serial.print(dRight); 
      Serial.println(F(" cm")); 
    }

    long dLeft = getDistanceAtAngle(leftAngle);
    Serial.print(F("[LEFT  ULTRA] "));
    if (dLeft < 0) Serial.println(F("No echo"));
    else { 
      Serial.print(dLeft); 
      Serial.println(F(" cm")); 
    }

    myservo.write(centerAngle);
    delay(250);

    if (dRight < 0 && dLeft < 0) {
      Serial.println(F("No safe direction → STOP"));
      stopCar();
    } 
    else if (dRight > dLeft) {
      Serial.println(F("Decision: TURN RIGHT"));
      rightCar();
      delay(400);
      stopCar();
    } 
    else if (dLeft > dRight) {
      Serial.println(F("Decision: TURN LEFT"));
      leftCar();
      delay(400);
      stopCar();
    } 
    else {
      Serial.println(F("Equal distances → STOP"));
      stopCar();
    }

    delay(250);
    return;
  }

  // (3) If HuskyLens is OFF → basic forward movement
  if (!huskyOK) {
    Serial.println(F("Husky OFF → Simple FORWARD"));
    forwardCar();
    delay(150);
    return;
  }

  // (4) HuskyLens object tracking logic
  if (!huskylens.request()) {
    Serial.println(F("Husky request failed → FORWARD"));
    forwardCar();
    delay(150);
    return;
  }

  if (!huskylens.isLearned()) {
    Serial.println(F("No learned objects → FORWARD"));
    forwardCar();
    delay(150);
    return;
  }

  if (!huskylens.available()) {
    Serial.println(F("No object detected → FORWARD"));
    forwardCar();
    delay(150);
    return;
  }

  HUSKYLENSResult target;
  bool found = false;

  while (huskylens.available()) {
    HUSKYLENSResult r = huskylens.read();
    if (r.command == COMMAND_RETURN_BLOCK && r.ID == TARGET_ID) {
      target = r;
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println(F("Target NOT found → FORWARD"));
    forwardCar();
    delay(150);
    return;
  }

  Serial.println(F("===== HUSKY DETECTED ====="));
  Serial.print(F("xCenter: ")); Serial.println(target.xCenter);
  Serial.print(F("yCenter: ")); Serial.println(target.yCenter);

  int centerX = 160;
  int deadZone = 40;
  int x = target.xCenter;

  if (x < centerX - deadZone) {
    Serial.println(F("CMD: LEFT (Target on left)"));
    leftCar();
  } 
  else if (x > centerX + deadZone) {
    Serial.println(F("CMD: RIGHT (Target on right)"));
    rightCar();
  } 
  else {
    Serial.println(F("CMD: FORWARD (Target centered)"));
    forwardCar();
  }

  delay(150);
}