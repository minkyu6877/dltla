/*
 * Interactive DW1000 anchor antenna-delay calibration.
 *
 * Hardware: Makerfabs ESP32_UWB (DW1000)
 * Serial:   115200 baud, Newline
 *
 * Commands:
 *   A,1       Save anchor number 1..4 and restart
 *   D,3.000   Set measured antenna-to-antenna distance in metres
 *   S         Start automatic calibration
 *   R         Reset the current calibration run
 *   P         Print current settings
 *
 * The anchor number and final delay are stored in ESP32 NVS. The same sketch
 * can therefore be uploaded to all four anchors without source edits.
 */

#include <Preferences.h>
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

const uint8_t SPI_SCK = 18;
const uint8_t SPI_MISO = 19;
const uint8_t SPI_MOSI = 23;

const uint16_t REFERENCE_TAG_SHORT_ADDRESS = 0x007D;
const uint16_t DEFAULT_ANTENNA_DELAY = 16600;
const uint16_t MIN_ANTENNA_DELAY = 10000;
const uint16_t MAX_ANTENNA_DELAY = 25000;

const int CALIBRATION_SAMPLES = 24;
const int VERIFICATION_SAMPLES = 100;
const int SETTLE_RANGES = 6;
const int INITIAL_DELAY_STEP = 128;
const int MAX_CALIBRATION_ITERATIONS = 28;
const float FINISH_ERROR_M = 0.010f;
const unsigned long RANGE_WARNING_MS = 5000;

enum class RunState {
  IDLE,
  CAL_SETTLING,
  CAL_SAMPLING,
  VERIFY_SETTLING,
  VERIFY_SAMPLING,
  COMPLETE,
  ERROR_STATE,
};

struct SampleStats {
  float mean;
  float stddev;
  float minimum;
  float maximum;
};

Preferences preferences;
RunState runState = RunState::IDLE;

uint8_t anchorNumber = 1;
char anchorAddress[24];
float targetDistanceM = 3.000f;
uint16_t currentDelay = DEFAULT_ANTENNA_DELAY;
uint16_t bestDelay = DEFAULT_ANTENNA_DELAY;
float bestAbsError = 999.0f;

int delayStep = INITIAL_DELAY_STEP;
int iteration = 0;
int settleRangesRemaining = 0;
int sampleCount = 0;
int stepOneRounds = 0;
float previousError = 0.0f;
bool previousErrorValid = false;
float samples[VERIFICATION_SAMPLES];

unsigned long lastRangeMs = 0;
unsigned long lastWarningMs = 0;
char serialBuffer[64];
size_t serialLength = 0;

void buildAnchorAddress() {
  const uint8_t shortId = 0x80 + anchorNumber;
  snprintf(anchorAddress, sizeof(anchorAddress),
           "%02X:00:5B:D5:A9:9A:E2:9C", shortId);
}

const char* stateName() {
  switch (runState) {
    case RunState::IDLE: return "IDLE";
    case RunState::CAL_SETTLING: return "CAL_SETTLING";
    case RunState::CAL_SAMPLING: return "CAL_SAMPLING";
    case RunState::VERIFY_SETTLING: return "VERIFY_SETTLING";
    case RunState::VERIFY_SAMPLING: return "VERIFY_SAMPLING";
    case RunState::COMPLETE: return "COMPLETE";
    default: return "ERROR";
  }
}

void insertionSort(float* values, int count) {
  for (int i = 1; i < count; ++i) {
    const float value = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }
}

SampleStats calculateStats(const float* values, int count, bool trimEdges) {
  float sorted[VERIFICATION_SAMPLES];
  for (int i = 0; i < count; ++i) sorted[i] = values[i];
  insertionSort(sorted, count);

  int trim = trimEdges ? count / 10 : 0;
  if (count - 2 * trim < 3) trim = 0;
  const int first = trim;
  const int last = count - trim;

  float sum = 0.0f;
  for (int i = first; i < last; ++i) sum += sorted[i];
  const float mean = sum / static_cast<float>(last - first);

  float variance = 0.0f;
  for (int i = first; i < last; ++i) {
    const float delta = sorted[i] - mean;
    variance += delta * delta;
  }
  variance /= static_cast<float>(last - first);

  SampleStats result;
  result.mean = mean;
  result.stddev = sqrtf(variance);
  result.minimum = sorted[0];
  result.maximum = sorted[count - 1];
  return result;
}

void printSettings() {
  Serial.print("CAL_READY,anchor=");
  Serial.print(anchorNumber);
  Serial.print(",address=");
  Serial.print(anchorAddress);
  Serial.print(",delay=");
  Serial.print(currentDelay);
  Serial.print(",target_m=");
  Serial.print(targetDistanceM, 3);
  Serial.print(",state=");
  Serial.println(stateName());
}

void resetRun() {
  runState = RunState::IDLE;
  delayStep = INITIAL_DELAY_STEP;
  iteration = 0;
  settleRangesRemaining = 0;
  sampleCount = 0;
  stepOneRounds = 0;
  previousError = 0.0f;
  previousErrorValid = false;
  bestDelay = currentDelay;
  bestAbsError = 999.0f;
  DW1000.setAntennaDelay(currentDelay);
  Serial.println("CAL_RESET");
  printSettings();
}

void beginSampling(RunState settlingState) {
  runState = settlingState;
  settleRangesRemaining = SETTLE_RANGES;
  sampleCount = 0;
}

void startVerification() {
  currentDelay = bestDelay;
  DW1000.setAntennaDelay(currentDelay);
  beginSampling(RunState::VERIFY_SETTLING);
  Serial.print("CAL_VERIFY_START,delay=");
  Serial.println(currentDelay);
}

void finishVerification() {
  const SampleStats stats =
      calculateStats(samples, VERIFICATION_SAMPLES, true);
  const float error = stats.mean - targetDistanceM;

  preferences.begin("uwb_cal", false);
  preferences.putUShort("ant_delay", currentDelay);
  preferences.end();

  runState = RunState::COMPLETE;
  Serial.print("CAL_RESULT,anchor=");
  Serial.print(anchorNumber);
  Serial.print(",address=");
  Serial.print(anchorAddress);
  Serial.print(",delay=");
  Serial.print(currentDelay);
  Serial.print(",target_m=");
  Serial.print(targetDistanceM, 3);
  Serial.print(",mean_m=");
  Serial.print(stats.mean, 3);
  Serial.print(",std_m=");
  Serial.print(stats.stddev, 3);
  Serial.print(",min_m=");
  Serial.print(stats.minimum, 3);
  Serial.print(",max_m=");
  Serial.print(stats.maximum, 3);
  Serial.print(",error_m=");
  Serial.print(error, 3);
  Serial.print(",samples=");
  Serial.println(VERIFICATION_SAMPLES);
}

void evaluateCalibrationStep() {
  const SampleStats stats =
      calculateStats(samples, CALIBRATION_SAMPLES, true);
  const float error = stats.mean - targetDistanceM;
  const float absError = fabsf(error);

  if (absError < bestAbsError) {
    bestAbsError = absError;
    bestDelay = currentDelay;
  }

  if (previousErrorValid && error * previousError < 0.0f) {
    delayStep = max(1, delayStep / 2);
  }

  Serial.print("CAL_STEP,iteration=");
  Serial.print(iteration);
  Serial.print(",delay=");
  Serial.print(currentDelay);
  Serial.print(",mean_m=");
  Serial.print(stats.mean, 3);
  Serial.print(",std_m=");
  Serial.print(stats.stddev, 3);
  Serial.print(",error_m=");
  Serial.print(error, 3);
  Serial.print(",step=");
  Serial.print(delayStep);
  Serial.print(",best_delay=");
  Serial.println(bestDelay);

  iteration++;
  if (delayStep <= 1) stepOneRounds++;

  if ((absError <= FINISH_ERROR_M && delayStep <= 2) ||
      stepOneRounds >= 4 || iteration >= MAX_CALIBRATION_ITERATIONS) {
    startVerification();
    return;
  }

  previousError = error;
  previousErrorValid = true;

  long nextDelay = static_cast<long>(currentDelay);
  nextDelay += error > 0.0f ? delayStep : -delayStep;
  if (nextDelay < MIN_ANTENNA_DELAY || nextDelay > MAX_ANTENNA_DELAY) {
    runState = RunState::ERROR_STATE;
    Serial.println("CAL_ERROR,reason=delay_out_of_range");
    return;
  }

  currentDelay = static_cast<uint16_t>(nextDelay);
  DW1000.setAntennaDelay(currentDelay);
  beginSampling(RunState::CAL_SETTLING);
}

void startCalibration() {
  if (!isfinite(targetDistanceM) || targetDistanceM < 0.5f ||
      targetDistanceM > 20.0f) {
    Serial.println("CAL_ERROR,reason=target_must_be_0.5_to_20.0");
    return;
  }

  delayStep = INITIAL_DELAY_STEP;
  iteration = 0;
  stepOneRounds = 0;
  previousErrorValid = false;
  bestDelay = currentDelay;
  bestAbsError = 999.0f;
  DW1000.setAntennaDelay(currentDelay);
  beginSampling(RunState::CAL_SETTLING);

  Serial.print("CAL_START,anchor=");
  Serial.print(anchorNumber);
  Serial.print(",target_m=");
  Serial.print(targetDistanceM, 3);
  Serial.print(",start_delay=");
  Serial.println(currentDelay);
}

void setAnchorNumber(int requested) {
  if (requested < 1 || requested > 4) {
    Serial.println("CAL_ERROR,reason=anchor_must_be_1_to_4");
    return;
  }

  preferences.begin("uwb_cal", false);
  preferences.putUChar("anchor_id", static_cast<uint8_t>(requested));
  preferences.end();
  Serial.print("CAL_RESTART,new_anchor=");
  Serial.println(requested);
  delay(300);
  ESP.restart();
}

void processCommand(char* command) {
  while (*command == ' ' || *command == '\t') command++;
  if (command[0] == '\0') return;

  if ((command[0] == 'A' || command[0] == 'a') && command[1] == ',') {
    setAnchorNumber(atoi(command + 2));
    return;
  }

  if ((command[0] == 'D' || command[0] == 'd') && command[1] == ',') {
    const float requested = atof(command + 2);
    if (!isfinite(requested) || requested < 0.5f || requested > 20.0f) {
      Serial.println("CAL_ERROR,reason=target_must_be_0.5_to_20.0");
      return;
    }
    targetDistanceM = requested;
    Serial.print("CAL_ACK,target_m=");
    Serial.println(targetDistanceM, 3);
    return;
  }

  if ((command[0] == 'S' || command[0] == 's') && command[1] == '\0') {
    startCalibration();
  } else if ((command[0] == 'R' || command[0] == 'r') &&
             command[1] == '\0') {
    resetRun();
  } else if ((command[0] == 'P' || command[0] == 'p') &&
             command[1] == '\0') {
    printSettings();
  } else {
    Serial.println("CAL_ERROR,reason=unknown_command,use_A_D_S_R_P");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      serialBuffer[serialLength] = '\0';
      processCommand(serialBuffer);
      serialLength = 0;
      continue;
    }
    if (serialLength + 1 < sizeof(serialBuffer)) {
      serialBuffer[serialLength++] = ch;
    } else {
      serialLength = 0;
      Serial.println("CAL_ERROR,reason=command_too_long");
    }
  }
}

void newRange() {
  DW1000Device* device = DW1000Ranging.getDistantDevice();
  if (device == nullptr) return;
  if (device->getShortAddress() != REFERENCE_TAG_SHORT_ADDRESS) return;

  const float rangeM = device->getRange();
  if (!isfinite(rangeM) || rangeM <= 0.0f || rangeM > 50.0f) return;
  lastRangeMs = millis();

  if (runState == RunState::CAL_SETTLING ||
      runState == RunState::VERIFY_SETTLING) {
    if (--settleRangesRemaining <= 0) {
      sampleCount = 0;
      runState = runState == RunState::CAL_SETTLING
                     ? RunState::CAL_SAMPLING
                     : RunState::VERIFY_SAMPLING;
    }
    return;
  }

  if (runState == RunState::CAL_SAMPLING) {
    samples[sampleCount++] = rangeM;
    if (sampleCount >= CALIBRATION_SAMPLES) evaluateCalibrationStep();
    return;
  }

  if (runState == RunState::VERIFY_SAMPLING) {
    samples[sampleCount++] = rangeM;
    if (sampleCount >= VERIFICATION_SAMPLES) finishVerification();
    return;
  }

  static unsigned long lastIdlePrintMs = 0;
  const unsigned long now = millis();
  if (now - lastIdlePrintMs >= 500) {
    lastIdlePrintMs = now;
    Serial.print("CAL_RANGE,range_m=");
    Serial.print(rangeM, 3);
    Serial.print(",rx_dbm=");
    Serial.println(device->getRXPower(), 1);
  }
}

void newDevice(DW1000Device* device) {
  Serial.print("CAL_DEVICE_ADDED,short=0x");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("CAL_DEVICE_LOST,short=0x");
  Serial.println(device->getShortAddress(), HEX);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  preferences.begin("uwb_cal", true);
  anchorNumber = preferences.getUChar("anchor_id", 1);
  currentDelay = preferences.getUShort(
      "ant_delay", DEFAULT_ANTENNA_DELAY);
  preferences.end();
  if (anchorNumber < 1 || anchorNumber > 4) anchorNumber = 1;
  if (currentDelay < MIN_ANTENNA_DELAY || currentDelay > MAX_ANTENNA_DELAY) {
    currentDelay = DEFAULT_ANTENNA_DELAY;
  }
  buildAnchorAddress();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  // The modified library preserves this value during network configuration.
  DW1000.setAntennaDelay(currentDelay);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsAnchor(
      anchorAddress, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  Serial.println("CAL_HELP,commands=A_1_to_4,D_distance_m,S,R,P");
  printSettings();
}

void loop() {
  DW1000Ranging.loop();
  readSerialCommands();

  const unsigned long now = millis();
  const bool running = runState == RunState::CAL_SETTLING ||
                       runState == RunState::CAL_SAMPLING ||
                       runState == RunState::VERIFY_SETTLING ||
                       runState == RunState::VERIFY_SAMPLING;
  if (running && now - lastRangeMs > RANGE_WARNING_MS &&
      now - lastWarningMs > RANGE_WARNING_MS) {
    lastWarningMs = now;
    Serial.println("CAL_WARNING,reason=no_reference_tag_range");
  }
}
