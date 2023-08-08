#include "pins.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS  115200
#define INTERVAL    500

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  pinMode(VOLTAGE_PIN, INPUT);

  Serial.println("Start");
}

static unsigned long timeout;

void loop() {
  // put your main code here, to run repeatedly:
  const unsigned long t0 = millis();
  if (t0 >= timeout) {
    timeout = t0 + INTERVAL;
    const int supply = analogRead(VOLTAGE_PIN);
    Serial.print("Supply ");
    Serial.print(supply);
    Serial.println();
  }
}
