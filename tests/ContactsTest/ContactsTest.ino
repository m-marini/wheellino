#include "Contacts.h"
#include "pins.h"

//#define DEBUG
#include "debug.h"

#define SERIAL_BPS  115200

#define INTERVAL 100

static ContactSensors sensors(FRONT_CONTACTS_PIN, REAR_CONTACTS_PIN);

void setup() {
  Serial.begin(SERIAL_BPS);
  delay(500);
  Serial.println("");
  sensors.begin();

  Serial.println("Start");
}

static boolean prevFront;
static boolean prevRear;

unsigned long timeout;

void loop() {
  // put your main code here, to run repeatedly:
  const unsigned long t0 = millis();
  if (t0 >= timeout) {
    timeout = t0 + INTERVAL;
    sensors.polling(t0);
    const boolean front = sensors.frontClear();
    const boolean rear = sensors.rearClear();
    if (front != prevFront || rear != prevRear) {
      prevFront = front;
      prevRear = rear;
      Serial.print(front ? "  front" : "X front");
      Serial.print(", ");
      Serial.print(rear ? "   rear" : "X rear");
      Serial.println();
    }
  }
}
