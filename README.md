Wheelly is a wheels robot.

The robot consits of

- Chassis with two lateral wheels driven by 2 x DC motors and a free wheel.
- 2 x IR speed sensors for the wheels
- Motor controller board L298N
- 12V DC rechargable Li battery
- Step down DC-DC 12V-5V converter
- Power supply module 3.3V
- MPU6050 gyroscope module
- ESP8266 Wifi module
- SG90 Microservo
- SR04 Ultrasonic proximity sensor
- 8 x microswitch contact sensors
- Arduino UNO main controller board

The Ardunio controller is driving all the units to allow the robot to collect the sensor data, compute the position relative the initial state, drive the proxcimity sensor direction, drive the robot for a specific direction and speed by feeding back from gyroscpe and speed sensors and comunicate with remote server via wifi sending status and receiving commands.

The wifi module led flashes depending on the comunication state:

- Slow flashing with double blink when acting as access point to "Wheelly" network SSID (no password)
- Slow flashing when acting as network point to configured network SSID
- Medium flashing when client connected and no activity in place
- Fast flashing when client connected and activity in place


Wheelly status led:

- Fixed red during MPU initialization
- Fixed yellow during servo and scanner initialization
- Single green blink when initialization completed

- Blinking blue when active with no obstacle
- Fast blinking green when there is a blocking obstacle in the back
- Fast blinking yellow when there is a blocking obstacle in the front side
- Fast blinking red when there are a full blocking obstacles
- Blinking cyan when there is a non-blocking obstacle near front
- Blinking magenta when there is a non-blocking obstacle in the very near front

## Datasheet

### Geometry 

- Track (distance between wheels): 136 mm
- Wheel diameter: 67 mm
- Wheel sensor pulses per root: 40
- Distance per pulse: 5.3 mm
- Direction rotation per pulse: 2.2 DEG

### Dinamics

- Wheel idle speed (pulses per second): 60 pps
- Wheel speed at full load (pulses per second): 20 pps
- Max wheel speed at full load at 12V power supply (pulses per second): 20 pps
- Linear idle speed: 0.318 m/s
- Linear speed at full load: 0.106 m/s
- Max direction angular idle speed: 264 DEG/s
- Max direction angular speed at full load: 88 DEG/s

## Motor controller

The motor controller provides power supply to the motor to keep the speed at desired value.

Assuming $ P(t) $ the power provided (-255, ..., 255) at instant t,
$ x(t) $ the desired speed value (pps),
$ \omega(t) $ the measured speed value (pps)

The supplyer power at next instant P(t+1) is
$ P(t+1) = P(t) + \alpha [x(t)- \omega(t)] $

$ \alpha $ is the feedbeck factor and should be the ratio between max power and max speed:
$ \alpha  \approx \frac{P_{max}}{\omega_{max}}  $.


