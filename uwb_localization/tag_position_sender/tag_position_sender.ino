/*
 * Four-anchor DW1000 tag position sender for Makerfabs ESP32_UWB.
 *
 * Arena coordinates (antenna centres):
 *   A1=(0,0), A2=(4,0), A3=(4,3), A4=(0,3) metres.
 * All anchor and tag antenna centres are at the same 0.10 m height.
 *
 * The tag solves its 2D position, prints a fixed-format serial packet, and
 * sends the same packet to the Raspberry Pi over Wi-Fi UDP port 4220.
 */

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include "secrets.h"

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;
const uint8_t SPI_SCK = 18;
const uint8_t SPI_MISO = 19;
const uint8_t SPI_MOSI = 23;

char TAG_ADDRESS[] = "7D:00:22:EA:82:60:3B:9C";
const uint16_t TAG_ANTENNA_DELAY = 16384;

const int ANCHOR_COUNT = 4;
const float ANCHOR_X_M[ANCHOR_COUNT] = {0.000f, 4.000f, 4.000f, 0.000f};
const float ANCHOR_Y_M[ANCHOR_COUNT] = {0.000f, 0.000f, 3.000f, 3.000f};
const float UWB_HEIGHT_M = 0.100f;

const uint16_t POSITION_PORT = 4220;
const uint16_t LOCAL_UDP_PORT = 4221;
const unsigned long RANGE_MAX_AGE_MS = 1200;
const unsigned long SEND_INTERVAL_MS = 100;
const float RANGE_FILTER_ALPHA = 0.35f;
const float POSITION_FILTER_ALPHA = 0.35f;
const float MAX_ACCEPTED_RMSE_M = 0.35f;

WiFiUDP udp;
IPAddress raspberryPiIp;
bool udpStarted = false;
unsigned long lastWiFiAttemptMs = 0;

float rangesM[ANCHOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
bool rangeInitialized[ANCHOR_COUNT] = {false, false, false, false};
unsigned long rangeUpdatedMs[ANCHOR_COUNT] = {0, 0, 0, 0};

float filteredX = 0.0f;
float filteredY = 0.0f;
bool positionInitialized = false;
unsigned long lastSendMs = 0;
uint32_t sequenceNumber = 0;

bool solvePosition(float& x, float& y, float& rmse) {
  const float x0 = ANCHOR_X_M[0];
  const float y0 = ANCHOR_Y_M[0];
  const float d0 = rangesM[0];

  float a[ANCHOR_COUNT - 1][2];
  float b[ANCHOR_COUNT - 1];
  for (int i = 1; i < ANCHOR_COUNT; ++i) {
    a[i - 1][0] = ANCHOR_X_M[i] - x0;
    a[i - 1][1] = ANCHOR_Y_M[i] - y0;
    b[i - 1] = d0 * d0 - rangesM[i] * rangesM[i]
             + ANCHOR_X_M[i] * ANCHOR_X_M[i]
             + ANCHOR_Y_M[i] * ANCHOR_Y_M[i]
             - x0 * x0 - y0 * y0;
  }

  float ata00 = 0.0f;
  float ata01 = 0.0f;
  float ata11 = 0.0f;
  float atb0 = 0.0f;
  float atb1 = 0.0f;
  for (int i = 0; i < ANCHOR_COUNT - 1; ++i) {
    ata00 += a[i][0] * a[i][0];
    ata01 += a[i][0] * a[i][1];
    ata11 += a[i][1] * a[i][1];
    atb0 += a[i][0] * b[i];
    atb1 += a[i][1] * b[i];
  }

  const float determinant = ata00 * ata11 - ata01 * ata01;
  if (fabsf(determinant) < 1.0e-6f) return false;

  const float inverse00 = ata11 / determinant;
  const float inverse01 = -ata01 / determinant;
  const float inverse11 = ata00 / determinant;
  x = 0.5f * (inverse00 * atb0 + inverse01 * atb1);
  y = 0.5f * (inverse01 * atb0 + inverse11 * atb1);

  float squaredError = 0.0f;
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    const float dx = x - ANCHOR_X_M[i];
    const float dy = y - ANCHOR_Y_M[i];
    const float predicted = sqrtf(dx * dx + dy * dy);
    const float residual = rangesM[i] - predicted;
    squaredError += residual * residual;
  }
  rmse = sqrtf(squaredError / static_cast<float>(ANCHOR_COUNT));
  return isfinite(x) && isfinite(y) && isfinite(rmse);
}

bool allRangesFresh(unsigned long now) {
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    if (!rangeInitialized[i] || now - rangeUpdatedMs[i] > RANGE_MAX_AGE_MS) {
      return false;
    }
  }
  return true;
}

void sendPosition(float rawX, float rawY, float rmse) {
  if (!positionInitialized) {
    filteredX = rawX;
    filteredY = rawY;
    positionInitialized = true;
  } else {
    filteredX += POSITION_FILTER_ALPHA * (rawX - filteredX);
    filteredY += POSITION_FILTER_ALPHA * (rawY - filteredY);
  }

  const bool insideArena = rawX >= -0.50f && rawX <= 4.50f
                        && rawY >= -0.50f && rawY <= 3.50f;
  const bool good = insideArena && rmse <= MAX_ACCEPTED_RMSE_M;
  const int rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;

  char packet[320];
  snprintf(
      packet, sizeof(packet),
      "UWB_POS,seq=%lu,tag_x_m=%.3f,tag_y_m=%.3f,raw_x_m=%.3f,raw_y_m=%.3f,"
      "rmse_m=%.3f,ranges_m=%.3f:%.3f:%.3f:%.3f,height_m=%.3f,"
      "wifi_rssi=%d,quality=%s",
      static_cast<unsigned long>(sequenceNumber++),
      filteredX, filteredY, rawX, rawY, rmse,
      rangesM[0], rangesM[1], rangesM[2], rangesM[3],
      UWB_HEIGHT_M, rssi, good ? "OK" : "CHECK");

  Serial.println(packet);
  if (WiFi.status() == WL_CONNECTED && udpStarted) {
    udp.beginPacket(raspberryPiIp, POSITION_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(packet), strlen(packet));
    udp.endPacket();
  }
}

void newRange() {
  DW1000Device* device = DW1000Ranging.getDistantDevice();
  if (device == nullptr) return;

  const int anchorNumber = device->getShortAddress() & 0x07;
  if (anchorNumber < 1 || anchorNumber > ANCHOR_COUNT) return;

  const float measured = device->getRange();
  if (!isfinite(measured) || measured < 0.15f || measured > 10.0f) return;

  const int index = anchorNumber - 1;
  if (!rangeInitialized[index]) {
    rangesM[index] = measured;
    rangeInitialized[index] = true;
  } else {
    rangesM[index] += RANGE_FILTER_ALPHA * (measured - rangesM[index]);
  }
  rangeUpdatedMs[index] = millis();

  const unsigned long now = millis();
  if (!allRangesFresh(now) || now - lastSendMs < SEND_INTERVAL_MS) return;

  float rawX = 0.0f;
  float rawY = 0.0f;
  float rmse = 0.0f;
  if (solvePosition(rawX, rawY, rmse)) {
    lastSendMs = now;
    sendPosition(rawX, rawY, rmse);
  }
}

void newDevice(DW1000Device* device) {
  Serial.print("UWB_ANCHOR_ADDED,address=0x");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  const int anchorNumber = device->getShortAddress() & 0x07;
  if (anchorNumber >= 1 && anchorNumber <= ANCHOR_COUNT) {
    rangeInitialized[anchorNumber - 1] = false;
  }
  Serial.print("UWB_ANCHOR_LOST,address=0x");
  Serial.println(device->getShortAddress(), HEX);
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiAttemptMs = millis();
  Serial.print("WIFI_CONNECTING,ssid=");
  Serial.println(WIFI_SSID);
}

void serviceWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udpStarted = udp.begin(LOCAL_UDP_PORT);
      Serial.print("WIFI_READY,ip=");
      Serial.print(WiFi.localIP());
      Serial.print(",pi=");
      Serial.print(raspberryPiIp);
      Serial.print(",udp_started=");
      Serial.println(udpStarted ? 1 : 0);
    }
    return;
  }

  if (udpStarted) {
    udp.stop();
    udpStarted = false;
  }
  const unsigned long now = millis();
  if (now - lastWiFiAttemptMs >= 5000) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWiFiAttemptMs = now;
    Serial.println("WIFI_RETRY");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!raspberryPiIp.fromString(RASPBERRY_PI_IP)) {
    Serial.println("CONFIG_ERROR,reason=invalid_raspberry_pi_ip");
  }
  startWiFi();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(TAG_ANTENNA_DELAY);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag(
      TAG_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  Serial.println("UWB_TAG_READY,anchors=4,arena_m=4.000:3.000,height_m=0.100");
}

void loop() {
  DW1000Ranging.loop();
  serviceWiFi();
}
