/*
 * DW1000 reference tag for calibrating ESP32_UWB anchors.
 *
 * Hardware: Makerfabs ESP32_UWB (DW1000)
 * Serial:   115200 baud
 *
 * Keep this tag's antenna delay fixed. Calibrate every anchor against this
 * same tag after the tag has been mounted on the robot in its final position.
 */

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

const uint8_t SPI_SCK = 18;
const uint8_t SPI_MISO = 19;
const uint8_t SPI_MOSI = 23;

char TAG_ADDRESS[] = "7D:00:22:EA:82:60:3B:9C";
const uint16_t REFERENCE_TAG_ANTENNA_DELAY = 16384;

unsigned long lastStatusMs = 0;
unsigned long rangeCount = 0;

void newRange() {
  DW1000Device* device = DW1000Ranging.getDistantDevice();
  if (device == nullptr) return;

  const float rangeM = device->getRange();
  const float rxPowerDbm = device->getRXPower();
  if (!isfinite(rangeM) || rangeM <= 0.0f || rangeM > 50.0f) return;

  rangeCount++;
  Serial.print("TAG_RANGE,anchor=0x");
  Serial.print(device->getShortAddress(), HEX);
  Serial.print(",range_m=");
  Serial.print(rangeM, 3);
  Serial.print(",rx_dbm=");
  Serial.println(rxPowerDbm, 1);
}

void newDevice(DW1000Device* device) {
  Serial.print("TAG_DEVICE_ADDED,anchor=0x");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("TAG_DEVICE_LOST,anchor=0x");
  Serial.println(device->getShortAddress(), HEX);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  // The modified library preserves this value during network configuration.
  DW1000.setAntennaDelay(REFERENCE_TAG_ANTENNA_DELAY);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag(
      TAG_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  Serial.print("TAG_READY,address=");
  Serial.print(TAG_ADDRESS);
  Serial.print(",delay=");
  Serial.println(REFERENCE_TAG_ANTENNA_DELAY);
}

void loop() {
  DW1000Ranging.loop();

  const unsigned long now = millis();
  if (now - lastStatusMs >= 5000) {
    lastStatusMs = now;
    Serial.print("TAG_STATUS,ranges=");
    Serial.println(rangeCount);
  }
}
