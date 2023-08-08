/*
   Test of proxy sersor SR04.

   The test measures the distance from the sensor and
   displays the distance in cm. an change
*/
#include "Arduino.h"
#include "pins.h"

#include "SR04.h"

#define SERIAL_BPS  115200

static unsigned int distance;

SR04Class SR04(TRIGGER_PIN, ECHO_PIN);

void setup() {
  Serial.begin(SERIAL_BPS);
  Serial.println("");
  SR04.onSample(handleSample);
  Serial.println("Starting ...");
  SR04.begin();
  SR04.start();
  Serial.println("Started");
}

void loop() {
  SR04.polling();
}

static void handleSample(void* , const unsigned long time) {
  const unsigned int echoDistance = time * 100 / 5887;
  if (distance != echoDistance) {
    distance = echoDistance;
    Serial.print("Distance ");
    Serial.print(distance);
    Serial.print(" cm");
    Serial.println();
  }
  SR04.start();
}
