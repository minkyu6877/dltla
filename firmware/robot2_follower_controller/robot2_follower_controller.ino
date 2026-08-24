/*
 * Robot 2 follower: mecanum drive + leader-gap measurement + MPU6050 telemetry.
 *
 * UDP command port 4210:
 *   PING
 *   STOP
 *   V,vx,vy,w,state
 *
 * Sensor behaviour:
 *   - The front HC-SR04 distance is reported as the gap to Robot 1.
 *   - Normal following distance never blocks motion.
 *   - Only an 8 cm last-resort collision guard stops the motors. Ultrasound
 *     alone cannot prove whether the target is Robot 1 or another object.
 *   - Loss of valid drive commands for 1 second also stops the motors.
 *
 * IMPORTANT: HC-SR04 ECHO is 5 V. Never connect it directly to ESP32 GPIO12.
 * Use a divider: ECHO -- 1 kOhm -- GPIO12 -- 2 kOhm -- GND.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "secrets.h"

// Copy secrets.example.h to secrets.h and enter the hotspot credentials.
const uint8_t ROBOT_ID = 2;
const char* FIRMWARE_VERSION = "2026-08-21-r2-follow-retreat-v2";

const uint16_t COMMAND_PORT = 4210;
const uint16_t STATUS_PORT = 4212;
const unsigned long COMMAND_TIMEOUT_MS = 1000;
const unsigned long STATUS_INTERVAL_MS = 250;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

WiFiUDP udp;
bool udpStarted = false;
bool controllerKnown = false;
IPAddress controllerIp;

// BTS7960 pins from the supplied schematic.
const int FL_RPWM = 25;
const int FL_LPWM = 26;
const int FR_RPWM = 27;
const int FR_LPWM = 14;
const int RL_RPWM = 32;
const int RL_LPWM = 33;
const int RR_RPWM = 18;
const int RR_LPWM = 19;

// Encoder pins.
const int ENC_FL_A = 13;
const int ENC_FL_B = 23;
const int ENC_FR_A = 16;
const int ENC_FR_B = 17;
const int ENC_RL_A = 21;
const int ENC_RL_B = 22;
const int ENC_RR_A = 34;
const int ENC_RR_B = 35;

// Robot 2 direction calibration already verified by the physical forward test.
const int MOTOR_INV_FL = 1;
const int MOTOR_INV_FR = 1;
const int MOTOR_INV_RL = 1;
const int MOTOR_INV_RR = 1;
const int ENC_INV_FL = 1;
const int ENC_INV_FR = -1;
const int ENC_INV_RL = 1;
const int ENC_INV_RR = -1;

const float ENCODER_COUNTS_PER_OUTPUT_REV = 660.0f;
const float MOTOR_REFERENCE_RPM = 300.0f;
const float MAX_TARGET_RPM = 180.0f;
const unsigned long CONTROL_INTERVAL_MS = 50;
const float KP = 0.0060f;
const float KI = 0.0015f;
const float KD = 0.0000f;
const float INTEGRAL_LIMIT = 60.0f;
const float RPM_DEADBAND = 3.0f;
const float MAX_PWM = 0.60f;
const float MAX_PWM_CHANGE_PER_CONTROL = 0.08f;

// HC-SR04: monitor the gap to Robot 1. 8 cm is only a final collision guard.
const int ULTRASONIC_TRIG = 2;
const int ULTRASONIC_ECHO = 12;
const float OBSTACLE_STOP_CM = 8.0f;
const float OBSTACLE_CLEAR_CM = 15.0f;
// Use a different period from Robot 1 to reduce repeated ultrasonic crosstalk.
const unsigned long ULTRASONIC_INTERVAL_MS = 97;
const unsigned long ULTRASONIC_TIMEOUT_US = 25000;
const int OBSTACLE_CONFIRM_SAMPLES = 2;
const int CLEAR_CONFIRM_SAMPLES = 3;

// MPU6050 I2C pins from the supplied schematic.
const int IMU_SDA = 4;
const int IMU_SCL = 5;
const uint8_t MPU6050_ADDRESS = 0x68;
const unsigned long IMU_INTERVAL_US = 20000;  // 50 Hz

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
unsigned long lastImuUs = 0;

float distanceCm = 0.0f;
bool distanceValid = false;
bool ultrasonicVerified = false;
bool obstacleLatched = false;
int obstacleSamples = 0;
int clearSamples = 0;

bool imuOk = false;
bool imuAnglesInitialized = false;
float accelXG = 0.0f;
float accelYG = 0.0f;
float accelZG = 0.0f;
float gyroXDps = 0.0f;
float gyroYDps = 0.0f;
float gyroZDps = 0.0f;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
float imuTempC = 0.0f;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float slewPwm(float current, float requested) {
  float change = clampFloat(
    requested - current,
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

float updateOnePid(
  float target,
  float actual,
  float& integral,
  float& previousError,
  float dt
) {
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

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMpuRaw(
  int16_t& ax,
  int16_t& ay,
  int16_t& az,
  int16_t& temperature,
  int16_t& gx,
  int16_t& gy,
  int16_t& gz
) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t received = Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14, true);
  if (received != 14 || Wire.available() < 14) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  temperature = (int16_t)((Wire.read() << 8) | Wire.read());
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

bool initializeImu() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  if (!writeMpuRegister(0x6B, 0x00)) return false;  // Wake MPU6050.
  writeMpuRegister(0x1A, 0x03);                    // DLPF about 44 Hz.
  writeMpuRegister(0x1B, 0x00);                    // Gyro +/-250 dps.
  writeMpuRegister(0x1C, 0x00);                    // Accel +/-2 g.
  delay(100);

  // Robot must remain flat and still during this gyro-bias calibration.
  const int samples = 300;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  int goodSamples = 0;
  for (int index = 0; index < samples; ++index) {
    int16_t ax, ay, az, temperature, gx, gy, gz;
    if (readMpuRaw(ax, ay, az, temperature, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      goodSamples++;
    }
    delay(3);
  }
  if (goodSamples < samples * 9 / 10) return false;
  gyroBiasX = ((float)sumX / goodSamples) / 131.0f;
  gyroBiasY = ((float)sumY / goodSamples) / 131.0f;
  gyroBiasZ = ((float)sumZ / goodSamples) / 131.0f;
  lastImuUs = micros();
  return true;
}

void updateImu() {
  if (!imuOk) return;
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastImuUs;
  if (elapsedUs < IMU_INTERVAL_US) return;
  lastImuUs = nowUs;

  int16_t rawAx, rawAy, rawAz, rawTemperature, rawGx, rawGy, rawGz;
  if (!readMpuRaw(rawAx, rawAy, rawAz, rawTemperature, rawGx, rawGy, rawGz)) {
    imuOk = false;
    pendingEvent = "IMU_READ_FAIL";
    return;
  }

  accelXG = rawAx / 16384.0f;
  accelYG = rawAy / 16384.0f;
  accelZG = rawAz / 16384.0f;
  gyroXDps = rawGx / 131.0f - gyroBiasX;
  gyroYDps = rawGy / 131.0f - gyroBiasY;
  gyroZDps = rawGz / 131.0f - gyroBiasZ;
  imuTempC = rawTemperature / 340.0f + 36.53f;

  float accelRoll = atan2(accelYG, accelZG) * 180.0f / PI;
  float accelPitch = atan2(-accelXG, sqrt(accelYG * accelYG + accelZG * accelZG)) * 180.0f / PI;
  float dt = clampFloat((float)elapsedUs / 1000000.0f, 0.001f, 0.10f);
  if (!imuAnglesInitialized) {
    rollDeg = accelRoll;
    pitchDeg = accelPitch;
    yawDeg = 0.0f;
    imuAnglesInitialized = true;
  } else {
    rollDeg = 0.98f * (rollDeg + gyroXDps * dt) + 0.02f * accelRoll;
    pitchDeg = 0.98f * (pitchDeg + gyroYDps * dt) + 0.02f * accelPitch;
    yawDeg += gyroZDps * dt;
    if (yawDeg > 180.0f) yawDeg -= 360.0f;
    if (yawDeg < -180.0f) yawDeg += 360.0f;
  }
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
    distanceValid = false;  // No echo normally means the closest target is out of range.
    obstacleSamples = 0;
    if (obstacleLatched && !robotIsMoving()) {
      clearSamples++;
      if (clearSamples >= CLEAR_CONFIRM_SAMPLES) {
        obstacleLatched = false;
        currentState = "STOP";
        pendingEvent = "OBSTACLE_CLEARED";
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
      pendingEvent = "FOLLOWER_COLLISION_GUARD_8CM";
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
      pendingEvent = "OBSTACLE_CLEARED";
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
  message += ",target_rpm=" + String(targetFL, 1) + ":" + String(targetFR, 1) + ":";
  message += String(targetRL, 1) + ":" + String(targetRR, 1);
  message += ",rpm=" + String(rpmFL, 1) + ":" + String(rpmFR, 1) + ":";
  message += String(rpmRL, 1) + ":" + String(rpmRR, 1);
  message += ",drive_pwm=" + String(pwmFL * MOTOR_INV_FL, 2) + ":";
  message += String(pwmFR * MOTOR_INV_FR, 2) + ":";
  message += String(pwmRL * MOTOR_INV_RL, 2) + ":";
  message += String(pwmRR * MOTOR_INV_RR, 2);
  message += ",distance_cm=";
  if (distanceValid) {
    message += String(distanceCm, 1);
  } else {
    message += "NA";
  }
  message += ",sensor_role=FOLLOW_GAP";
  message += ",us_verified=" + String(ultrasonicVerified ? 1 : 0);
  message += ",stop_cm=" + String(OBSTACLE_STOP_CM, 1);
  message += ",obstacle=" + String(obstacleLatched ? 1 : 0);
  message += ",imu_ok=" + String(imuOk ? 1 : 0);
  message += ",accel_g=" + String(accelXG, 3) + ":" + String(accelYG, 3) + ":" + String(accelZG, 3);
  message += ",gyro_dps=" + String(gyroXDps, 2) + ":" + String(gyroYDps, 2) + ":" + String(gyroZDps, 2);
  message += ",att_deg=" + String(rollDeg, 1) + ":" + String(pitchDeg, 1) + ":" + String(yawDeg, 1);
  message += ",imu_temp_c=" + String(imuTempC, 1);
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
      sendStatus("MOVE_BLOCKED");
      return;
    }
    if (requestedVx > 0.01f && distanceValid && distanceCm <= OBSTACLE_STOP_CM) {
      obstacleLatched = true;
      stopMotors("COLLISION_GUARD_STOP");
      sendStatus("FOLLOWER_TOO_CLOSE_8CM");
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
  char buffer[256];
  int length = udp.read(buffer, sizeof(buffer) - 1);
  if (length <= 0) return;
  buffer[length] = '\0';
  String message(buffer);
  message.trim();
  handlePacket(message);
}

void setupPins() {
  const int motorPins[] = {
    FL_RPWM, FL_LPWM, FR_RPWM, FR_LPWM,
    RL_RPWM, RL_LPWM, RR_RPWM, RR_LPWM
  };
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
  pinMode(ENC_RR_A, INPUT);  // GPIO34/35 require external pull-ups.
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
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udpStarted = udp.begin(COMMAND_PORT);
      currentState = "READY";
      Serial.println("Robot 2 IP: " + WiFi.localIP().toString());
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
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setupPins();
  Serial.println("Keep robot 2 flat and still: calibrating MPU6050...");
  imuOk = initializeImu();
  Serial.println(imuOk ? "MPU6050 ready" : "MPU6050 not detected");
  lastControlMs = millis();
  lastValidCommandMs = millis();
  lastStatusMs = millis();
  lastUltrasonicMs = millis();
  startWifi();
}

void loop() {
  maintainWifi();
  updateImu();
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
