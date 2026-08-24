/*
 * Robot 1 leader: mecanum drive + 20 cm front-obstacle stop.
 *
 * Raspberry Pi packet format (UDP 4210):
 *   PING
 *   STOP
 *   V,vx,vy,w,state
 *
 * vx: forward(+)/backward(-), vy: left(+)/right(-), w: CCW(+)/CW(-)
 * Each value is normalized to -1.0 ... 1.0.
 *
 * HC-SR04 ECHO is 5 V. Use the existing 1 kOhm / 2 kOhm divider before GPIO12.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"

// -----------------------------------------------------------------------------
// This sketch is calibrated for robot 1 only.
// Copy secrets.example.h to secrets.h and enter the hotspot credentials.
// -----------------------------------------------------------------------------
const uint8_t ROBOT_ID = 1;
const char* FIRMWARE_VERSION = "2026-08-21-r1-leader-retreat-v2";

const uint16_t COMMAND_PORT = 4210;
const uint16_t STATUS_PORT = 4212;

// A moving robot stops if valid V packets stop arriving for this long.
const unsigned long COMMAND_TIMEOUT_MS = 1000;
const unsigned long STATUS_INTERVAL_MS = 250;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

WiFiUDP udp;
bool udpStarted = false;
bool controllerKnown = false;
IPAddress controllerIp;

// -----------------------------------------------------------------------------
// BTS7960 pins from the supplied KiCad schematic:
// FL: RPWM=25, LPWM=26 / FR: RPWM=27, LPWM=14
// RL: RPWM=32, LPWM=33 / RR: RPWM=18, LPWM=19
// Tie each BTS7960 R_EN and L_EN HIGH. All controller grounds must be common.
// -----------------------------------------------------------------------------
const int FL_RPWM = 25;
const int FL_LPWM = 26;
const int FR_RPWM = 27;
const int FR_LPWM = 14;
const int RL_RPWM = 32;
const int RL_LPWM = 33;
const int RR_RPWM = 18;
const int RR_LPWM = 19;

// Encoder pins
const int ENC_FL_A = 13;
const int ENC_FL_B = 23;
const int ENC_FR_A = 16;
const int ENC_FR_B = 17;
const int ENC_RL_A = 21;
const int ENC_RL_B = 22;
const int ENC_RR_A = 34;
const int ENC_RR_B = 35;

// Robot 1 test with the Robot 2 profile showed only FR backward/fast.
// Flip only Robot 1 FR; FL, RL and RR remain identical to Robot 2.
const int MOTOR_INV_FL = 1;
const int MOTOR_INV_FR = -1;
const int MOTOR_INV_RL = 1;
const int MOTOR_INV_RR = 1;

const int ENC_INV_FL = 1;
const int ENC_INV_FR = -1;
const int ENC_INV_RL = 1;
const int ENC_INV_RR = -1;

// JGB520 1:30, encoder 11 PPR, channel A CHANGE: 11 * 2 * 30 = 660.
// Measure and correct this value if the actual gear ratio/encoder differs.
const float ENCODER_COUNTS_PER_OUTPUT_REV = 660.0f;
const float MOTOR_REFERENCE_RPM = 300.0f;
const float MAX_TARGET_RPM = 180.0f;

const unsigned long CONTROL_INTERVAL_MS = 50;
const float KP = 0.0060f;
const float KI = 0.0015f;
const float KD = 0.0000f;
const float INTEGRAL_LIMIT = 60.0f;
const float RPM_DEADBAND = 3.0f;
const float MIN_PWM_TO_MOVE = 0.00f;
// Limit and ramp PWM while calibrating. This prevents one bad encoder channel
// from immediately driving a wheel at 100%.
const float MAX_PWM = 0.60f;
const float MAX_PWM_CHANGE_PER_CONTROL = 0.08f;

// Robot 1 is the leader. Its front sensor protects the whole convoy.
const int ULTRASONIC_TRIG = 2;
const int ULTRASONIC_ECHO = 12;
const float OBSTACLE_STOP_CM = 20.0f;
const float OBSTACLE_CLEAR_CM = 30.0f;
// Use a different period from Robot 2 to reduce repeated ultrasonic crosstalk.
const unsigned long ULTRASONIC_INTERVAL_MS = 83;
const unsigned long ULTRASONIC_TIMEOUT_US = 25000;
const int OBSTACLE_CONFIRM_SAMPLES = 2;
const int CLEAR_CONFIRM_SAMPLES = 3;

volatile long encCountFL = 0;
volatile long encCountFR = 0;
volatile long encCountRL = 0;
volatile long encCountRR = 0;

long previousCountFL = 0;
long previousCountFR = 0;
long previousCountRL = 0;
long previousCountRR = 0;

float rpmFL = 0.0f;
float rpmFR = 0.0f;
float rpmRL = 0.0f;
float rpmRR = 0.0f;

float targetFL = 0.0f;
float targetFR = 0.0f;
float targetRL = 0.0f;
float targetRR = 0.0f;

float pwmFL = 0.0f;
float pwmFR = 0.0f;
float pwmRL = 0.0f;
float pwmRR = 0.0f;

float integralFL = 0.0f;
float integralFR = 0.0f;
float integralRL = 0.0f;
float integralRR = 0.0f;

float previousErrorFL = 0.0f;
float previousErrorFR = 0.0f;
float previousErrorRL = 0.0f;
float previousErrorRR = 0.0f;

float commandVx = 0.0f;
float commandVy = 0.0f;
float commandW = 0.0f;

String currentState = "BOOT";
String pendingEvent = "";
unsigned long lastValidCommandMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastUltrasonicMs = 0;

float distanceCm = 0.0f;
bool distanceValid = false;
bool ultrasonicVerified = false;
bool obstacleLatched = false;
int obstacleSamples = 0;
int clearSamples = 0;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float slewPwm(float current, float requested) {
  float change = requested - current;
  change = clampFloat(
    change,
    -MAX_PWM_CHANGE_PER_CONTROL,
    MAX_PWM_CHANGE_PER_CONTROL
  );
  return current + change;
}

void driveBTS7960(int rpwmPin, int lpwmPin, float value) {
  value = clampFloat(value, -1.0f, 1.0f);
  int duty = constrain((int)(fabs(value) * 255.0f), 0, 255);

  if (value > 0.0f) {
    analogWrite(rpwmPin, duty);
    analogWrite(lpwmPin, 0);
  } else if (value < 0.0f) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, duty);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void applyMotorOutputs() {
  driveBTS7960(FL_RPWM, FL_LPWM, pwmFL * MOTOR_INV_FL);
  driveBTS7960(FR_RPWM, FR_LPWM, pwmFR * MOTOR_INV_FR);
  driveBTS7960(RL_RPWM, RL_LPWM, pwmRL * MOTOR_INV_RL);
  driveBTS7960(RR_RPWM, RR_LPWM, pwmRR * MOTOR_INV_RR);
}

void resetPid() {
  integralFL = integralFR = integralRL = integralRR = 0.0f;
  previousErrorFL = previousErrorFR = previousErrorRL = previousErrorRR = 0.0f;
}

void stopMotors(const String& state) {
  commandVx = commandVy = commandW = 0.0f;
  targetFL = targetFR = targetRL = targetRR = 0.0f;
  pwmFL = pwmFR = pwmRL = pwmRR = 0.0f;
  resetPid();
  applyMotorOutputs();
  currentState = state;
}

bool robotIsMoving() {
  return fabs(targetFL) > RPM_DEADBAND || fabs(targetFR) > RPM_DEADBAND ||
         fabs(targetRL) > RPM_DEADBAND || fabs(targetRR) > RPM_DEADBAND;
}

void IRAM_ATTR isrEncFL() {
  encCountFL += (digitalRead(ENC_FL_A) == digitalRead(ENC_FL_B) ? 1 : -1) * ENC_INV_FL;
}

void IRAM_ATTR isrEncFR() {
  encCountFR += (digitalRead(ENC_FR_A) == digitalRead(ENC_FR_B) ? 1 : -1) * ENC_INV_FR;
}

void IRAM_ATTR isrEncRL() {
  encCountRL += (digitalRead(ENC_RL_A) == digitalRead(ENC_RL_B) ? 1 : -1) * ENC_INV_RL;
}

void IRAM_ATTR isrEncRR() {
  encCountRR += (digitalRead(ENC_RR_A) == digitalRead(ENC_RR_B) ? 1 : -1) * ENC_INV_RR;
}

void setTargets(float vx, float vy, float w) {
  // Mecanum inverse kinematics, normalized.
  float fl = vx - vy - w;
  float fr = vx + vy + w;
  float rl = vx + vy - w;
  float rr = vx - vy + w;

  float largest = max(max(fabs(fl), fabs(fr)), max(fabs(rl), fabs(rr)));
  if (largest > 1.0f) {
    fl /= largest;
    fr /= largest;
    rl /= largest;
    rr /= largest;
  }

  targetFL = clampFloat(fl * MOTOR_REFERENCE_RPM, -MAX_TARGET_RPM, MAX_TARGET_RPM);
  targetFR = clampFloat(fr * MOTOR_REFERENCE_RPM, -MAX_TARGET_RPM, MAX_TARGET_RPM);
  targetRL = clampFloat(rl * MOTOR_REFERENCE_RPM, -MAX_TARGET_RPM, MAX_TARGET_RPM);
  targetRR = clampFloat(rr * MOTOR_REFERENCE_RPM, -MAX_TARGET_RPM, MAX_TARGET_RPM);
}

float updateOnePid(float target, float actual, float& integral, float& previousError, float dt) {
  if (fabs(target) < RPM_DEADBAND) {
    integral = 0.0f;
    previousError = 0.0f;
    return 0.0f;
  }

  float error = target - actual;
  integral = clampFloat(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = dt > 0.0f ? (error - previousError) / dt : 0.0f;
  previousError = error;

  float output = target / MOTOR_REFERENCE_RPM + KP * error + KI * integral + KD * derivative;
  if (MIN_PWM_TO_MOVE > 0.0f && fabs(output) < MIN_PWM_TO_MOVE) {
    output = output >= 0.0f ? MIN_PWM_TO_MOVE : -MIN_PWM_TO_MOVE;
  }
  return clampFloat(output, -MAX_PWM, MAX_PWM);
}

void updateControl() {
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastControlMs;
  if (elapsedMs < CONTROL_INTERVAL_MS) return;

  noInterrupts();
  long countFL = encCountFL;
  long countFR = encCountFR;
  long countRL = encCountRL;
  long countRR = encCountRR;
  interrupts();

  float elapsedMinutes = (float)elapsedMs / 60000.0f;
  rpmFL = ((float)(countFL - previousCountFL) / ENCODER_COUNTS_PER_OUTPUT_REV) / elapsedMinutes;
  rpmFR = ((float)(countFR - previousCountFR) / ENCODER_COUNTS_PER_OUTPUT_REV) / elapsedMinutes;
  rpmRL = ((float)(countRL - previousCountRL) / ENCODER_COUNTS_PER_OUTPUT_REV) / elapsedMinutes;
  rpmRR = ((float)(countRR - previousCountRR) / ENCODER_COUNTS_PER_OUTPUT_REV) / elapsedMinutes;

  previousCountFL = countFL;
  previousCountFR = countFR;
  previousCountRL = countRL;
  previousCountRR = countRR;

  float dt = (float)elapsedMs / 1000.0f;
  float requestedFL = updateOnePid(targetFL, rpmFL, integralFL, previousErrorFL, dt);
  float requestedFR = updateOnePid(targetFR, rpmFR, integralFR, previousErrorFR, dt);
  float requestedRL = updateOnePid(targetRL, rpmRL, integralRL, previousErrorRL, dt);
  float requestedRR = updateOnePid(targetRR, rpmRR, integralRR, previousErrorRR, dt);

  pwmFL = slewPwm(pwmFL, requestedFL);
  pwmFR = slewPwm(pwmFR, requestedFR);
  pwmRL = slewPwm(pwmRL, requestedRL);
  pwmRR = slewPwm(pwmRR, requestedRR);
  applyMotorOutputs();

  lastControlMs = now;
}

void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastUltrasonicMs < ULTRASONIC_INTERVAL_MS) return;
  lastUltrasonicMs = now;

  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);

  if (duration == 0) {
    distanceValid = false;
    obstacleSamples = 0;
    if (obstacleLatched && !robotIsMoving()) {
      clearSamples++;
      if (clearSamples >= CLEAR_CONFIRM_SAMPLES) {
        obstacleLatched = false;
        currentState = "STOP";
        pendingEvent = "LEADER_OBSTACLE_CLEARED";
        clearSamples = 0;
      }
    } else {
      clearSamples = 0;
    }
    return;
  }

  float measured = duration * 0.0343f * 0.5f;
  distanceValid = measured >= 2.0f && measured <= 400.0f;
  if (!distanceValid) {
    obstacleSamples = 0;
    clearSamples = 0;
    return;
  }

  distanceCm = measured;
  ultrasonicVerified = true;

  bool movingForward = robotIsMoving() && commandVx > 0.01f;
  if (!obstacleLatched && movingForward && distanceCm <= OBSTACLE_STOP_CM) {
    obstacleSamples++;
    if (obstacleSamples >= OBSTACLE_CONFIRM_SAMPLES) {
      obstacleLatched = true;
      stopMotors("OBSTACLE_STOP");
      pendingEvent = "LEADER_OBSTACLE_20CM";
      clearSamples = 0;
    }
  } else {
    obstacleSamples = 0;
  }

  if (obstacleLatched && !robotIsMoving() && distanceCm >= OBSTACLE_CLEAR_CM) {
    clearSamples++;
    if (clearSamples >= CLEAR_CONFIRM_SAMPLES) {
      obstacleLatched = false;
      currentState = "STOP";
      pendingEvent = "LEADER_OBSTACLE_CLEARED";
      clearSamples = 0;
    }
  } else {
    clearSamples = 0;
  }
}

void sendStatus(const String& event = "") {
  if (!controllerKnown || WiFi.status() != WL_CONNECTED || !udpStarted) return;

  String message = "STATUS,id=" + String(ROBOT_ID);
  message += ",fw=" + String(FIRMWARE_VERSION);
  message += ",state=" + currentState;
  message += ",ip=" + WiFi.localIP().toString();
  message += ",rssi=" + String(WiFi.RSSI());
  // All four-value fields are ordered FL:FR:RL:RR.
  message += ",target_rpm=" + String(targetFL, 1) + ":" + String(targetFR, 1) + ":";
  message += String(targetRL, 1) + ":" + String(targetRR, 1);
  message += ",rpm=" + String(rpmFL, 1) + ":" + String(rpmFR, 1) + ":";
  message += String(rpmRL, 1) + ":" + String(rpmRR, 1);
  message += ",drive_pwm=" + String(pwmFL * MOTOR_INV_FL, 2) + ":";
  message += String(pwmFR * MOTOR_INV_FR, 2) + ":";
  message += String(pwmRL * MOTOR_INV_RL, 2) + ":";
  message += String(pwmRR * MOTOR_INV_RR, 2);
  message += ",sensor_role=LEADER_OBSTACLE";
  message += ",distance_cm=";
  if (distanceValid) {
    message += String(distanceCm, 1);
  } else {
    message += "NA";
  }
  message += ",us_verified=" + String(ultrasonicVerified ? 1 : 0);
  message += ",stop_cm=" + String(OBSTACLE_STOP_CM, 1);
  message += ",obstacle=" + String(obstacleLatched ? 1 : 0);
  message += ",motor_inv=" + String(MOTOR_INV_FL) + ":" + String(MOTOR_INV_FR) + ":";
  message += String(MOTOR_INV_RL) + ":" + String(MOTOR_INV_RR);
  message += ",encoder_inv=" + String(ENC_INV_FL) + ":" + String(ENC_INV_FR) + ":";
  message += String(ENC_INV_RL) + ":" + String(ENC_INV_RR);
  if (event.length() > 0) message += ",event=" + event;

  udp.beginPacket(controllerIp, STATUS_PORT);
  udp.write((const uint8_t*)message.c_str(), message.length());
  udp.endPacket();
  Serial.println(message);
}

bool parseVelocityCommand(const String& message, float& vx, float& vy, float& w, String& state) {
  int comma1 = message.indexOf(',');
  int comma2 = message.indexOf(',', comma1 + 1);
  int comma3 = message.indexOf(',', comma2 + 1);
  int comma4 = message.indexOf(',', comma3 + 1);
  if (comma1 != 1 || comma2 < 0 || comma3 < 0 || comma4 < 0) return false;

  vx = message.substring(comma1 + 1, comma2).toFloat();
  vy = message.substring(comma2 + 1, comma3).toFloat();
  w = message.substring(comma3 + 1, comma4).toFloat();
  state = message.substring(comma4 + 1);
  state.trim();
  if (state.length() == 0) state = "MOVE";
  return true;
}

void handlePacket(const String& message) {
  controllerIp = udp.remoteIP();
  controllerKnown = true;
  Serial.println("[RX " + controllerIp.toString() + "] " + message);

  if (message == "PING") {
    sendStatus("PONG");
    return;
  }

  if (message == "STOP") {
    stopMotors(obstacleLatched ? "OBSTACLE_STOP" : "STOP");
    lastValidCommandMs = millis();
    sendStatus("STOP_ACK");
    return;
  }

  if (message.startsWith("V,")) {
    float vx, vy, w;
    String state;
    if (!parseVelocityCommand(message, vx, vy, w, state)) {
      sendStatus("BAD_COMMAND");
      return;
    }

    lastValidCommandMs = millis();
    float requestedVx = clampFloat(vx, -1.0f, 1.0f);
    float requestedVy = clampFloat(vy, -1.0f, 1.0f);
    float requestedW = clampFloat(w, -1.0f, 1.0f);
    bool reverseOnly = requestedVx < -0.01f &&
                       fabs(requestedVy) <= 0.01f &&
                       fabs(requestedW) <= 0.01f;
    if (obstacleLatched && !reverseOnly) {
      stopMotors("OBSTACLE_STOP");
      sendStatus("MOVE_BLOCKED_LEADER_OBSTACLE");
      return;
    }

    if (requestedVx > 0.01f && !ultrasonicVerified) {
      stopMotors("ULTRASONIC_NOT_VERIFIED");
      sendStatus("MOVE_BLOCKED_SENSOR_CHECK");
      return;
    }
    if (requestedVx > 0.01f && distanceValid && distanceCm <= OBSTACLE_STOP_CM) {
      obstacleLatched = true;
      stopMotors("OBSTACLE_STOP");
      sendStatus("LEADER_OBSTACLE_ALREADY_NEAR");
      return;
    }

    commandVx = requestedVx;
    commandVy = requestedVy;
    commandW = requestedW;
    currentState = obstacleLatched ? "RETREAT_REVERSE" : state;
    setTargets(commandVx, commandVy, commandW);
    return;
  }

  sendStatus("UNKNOWN_COMMAND");
}

void receiveUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  char buffer[192];
  int length = udp.read(buffer, sizeof(buffer) - 1);
  if (length <= 0) return;
  buffer[length] = '\0';

  String message(buffer);
  message.trim();
  handlePacket(message);
}

void setupPins() {
  const int motorPins[] = {FL_RPWM, FL_LPWM, FR_RPWM, FR_LPWM,
                           RL_RPWM, RL_LPWM, RR_RPWM, RR_LPWM};
  for (int pin : motorPins) {
    pinMode(pin, OUTPUT);
    analogWriteResolution(pin, 8);
    analogWriteFrequency(pin, 20000);
  }

  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP);
  pinMode(ENC_RL_B, INPUT_PULLUP);
  // GPIO34 and GPIO35 have no internal pull-up; use external 10 kOhm pull-ups.
  pinMode(ENC_RR_A, INPUT);
  pinMode(ENC_RR_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrEncFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrEncFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrEncRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrEncRR, CHANGE);

  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  pinMode(ULTRASONIC_ECHO, INPUT);

  stopMotors("BOOT");
}

void startWifi() {
  stopMotors("WIFI_CONNECT");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
  Serial.println("Connecting to Wi-Fi...");
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udpStarted = udp.begin(COMMAND_PORT);
      currentState = "READY";
      Serial.println("Robot " + String(ROBOT_ID) + " IP: " + WiFi.localIP().toString());
      Serial.println("UDP command port: " + String(COMMAND_PORT));
    }
    return;
  }

  if (udpStarted) {
    udp.stop();
    udpStarted = false;
  }
  stopMotors("WIFI_LOST");

  if (millis() - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttemptMs = millis();
    Serial.println("Retrying Wi-Fi...");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setupPins();
  lastControlMs = millis();
  lastValidCommandMs = millis();
  lastStatusMs = millis();
  lastUltrasonicMs = millis();
  startWifi();
}

void loop() {
  maintainWifi();
  updateUltrasonic();
  if (udpStarted) receiveUdp();

  updateControl();

  unsigned long now = millis();
  if (robotIsMoving() && now - lastValidCommandMs > COMMAND_TIMEOUT_MS) {
    stopMotors("TIMEOUT_STOP");
    pendingEvent = "TIMEOUT";
  }

  if (pendingEvent.length() > 0) {
    sendStatus(pendingEvent);
    pendingEvent = "";
  }

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    sendStatus();
  }
  delay(1);
}
