#include "pins.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS  115200
#define SAMPLE_INTERVAL   100
#define AVERAGE_INTERVAL  5000

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  pinMode(VOLTAGE_PIN, INPUT);

  Serial.println("Start");
}

static unsigned long sampleTimeout;
static unsigned long averageTimeout;
static long total;
static int count;

void loop() {
  // put your main code here, to run repeatedly:
  const unsigned long t0 = millis();
  if (t0 >= sampleTimeout) {
    total += analogRead(VOLTAGE_PIN);
    count++;
    sampleTimeout = t0 + SAMPLE_INTERVAL;
  }
  if (t0 >= averageTimeout) {
    if (count > 0) {
      int supply = total / count;
      Serial.print("Supply: ");
      Serial.print(supply);
      Serial.print(", count: ");
      Serial.print(count);
      Serial.println();
      total = 0;
      count = 0;
    }
    averageTimeout = t0 + AVERAGE_INTERVAL;
  }
}
