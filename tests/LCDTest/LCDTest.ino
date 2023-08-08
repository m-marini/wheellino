/*
   Test of LCD display

   The test show the display functions
*/
#include "Arduino.h"

#include "Display.h"

#define SERIAL_BPS  115200

#define DISPLAY_INTERVAL  10000

static DisplayClass Display;

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println("");
  Serial.println("Starting ...");

  Display.begin();

  Serial.println("Testing distance 5 cm");
  Display.clear();
  Display.showWiFiInfo("Testing distance 5 cm");
  Display.error(1);
  Display.block(FULL_BLOCK);
  Display.move(true);
  Display.distance(5);
  Display.connected(true);
  Display.activity();
  waitForTimeout(DISPLAY_INTERVAL);

  Serial.println("Testing distance 95 cm");
  Display.clear();
  Display.showWiFiInfo("Testing distance 95 cm");
  Display.error(2);
  Display.move(true);
  Display.block(FORWARD_BLOCK);
  Display.distance(95);
  Display.connected(true);
  Display.activity();
  waitForTimeout(DISPLAY_INTERVAL);

  Serial.println("Testing distance 345 cm");
  Display.clear();
  Display.showWiFiInfo("Testing distance 345 cm");
  Display.error(3);
  Display.move(true);
  Display.distance(345);
  Display.block(BACKWARD_BLOCK);
  Display.connected(true);
  Display.activity();
  waitForTimeout(DISPLAY_INTERVAL);

  Serial.println("Testing no distance");
  Display.clear();
  Display.showWiFiInfo("Testing no distance");
  Display.error(0);
  Display.move(false);
  Display.distance(-1);
  Display.block(NO_BLOCK);
  Display.connected(false);
  waitForTimeout(DISPLAY_INTERVAL);

  Display.error(0);
  Display.showWiFiInfo("");
  Display.distance(-1);
  Display.move(false);
  Display.block(NO_BLOCK);
  Display.connected(false);
  waitForTimeout(DISPLAY_INTERVAL);

  Serial.println("Completed");
  Display.clear();
}

void loop() {
  delay(1000);
}
static void waitForTimeout(const unsigned long interval) {
  unsigned long t0 = millis();
  const unsigned long timeout = t0 + interval;
  do {
    t0 = millis();
    Display.polling(t0);
  } while (t0 < timeout);
}
