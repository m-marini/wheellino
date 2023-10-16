#ifndef pins_h
#define pins_h

/*
  2  - critic: internal pull-down, must be left floating or LOW to enter flashing mode
  4  - internal pull-down
  5  - internal pull-up, outputs PWM signal at boot, strapping pin
  12 - critic: internal pull-down, boot fails if pulled high, strapping pin
  13 - SERVO_PIN
  14 - outputs PWM signal at boot
  15 - internal pull-up, outputs PWM signal at boot, strapping pin
  16 - LEFT_FORW_PIN
  17 - LEFT_BACK_PIN
  18 - RIGHT_FORW_PIN
  19 - RIGHT_BACK_PIN
  21 - SDA
  22 - SCL
  23 - TRIGGER_PIN
  25 - STATUS_LED_PIN
  26 - 
  27 - 
  32 - LEFT_PIN
  33 - RIGHT_PIN
  34 - ECHO_PIN - input only
  35 - FRONT_CONTACTS_PIN - input only
  36 - SP - REAR_CONTACTS_PIN - input only
  39 - SN - VOLTAGE_PIN - input only
*/

// The ADC pins

#define VOLTAGE_PIN         39

// The digital input pins

#define LEFT_PIN            32
#define RIGHT_PIN           33
#define ECHO_PIN            34
#define FRONT_CONTACTS_PIN  35
#define REAR_CONTACTS_PIN   36

// The PWM output pins

#define SERVO_PIN       13
#define LEFT_FORW_PIN   16
#define LEFT_BACK_PIN   17
#define RIGHT_FORW_PIN  18
#define RIGHT_BACK_PIN  19

// IC2 pins

#define SDA_PIN   21
#define SCL_PIN   22

// The digital output pins

#define TRIGGER_PIN     23
#define STATUS_LED_PIN  25

#endif
