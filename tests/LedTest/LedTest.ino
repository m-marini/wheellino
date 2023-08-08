#include "pins.h"
#include "Timer.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS  115200
#define INTERVAL    100

static Timer timer;

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  pinMode(STATUS_LED_PIN, OUTPUT);
  timer.interval(INTERVAL);
  timer.continuous(true);
  timer.onNext(handleTimer);

  Serial.println("Start");
  timer.start();
}

void loop() {
  timer.polling();
}

static void handleTimer(void*, const unsigned long n) {
  digitalWrite(STATUS_LED_PIN, (n % 2) != 0);
}
