A wheels robot (Wheelly)

The robot is consits of

- Chassis with two lateral wheels driven by two DC motors and a free wheel.
- 2 IR speed sensors for the wheels
- Motor controller board
- 12V DC rechargable Li battery
- Step down DC-DC 12V-5V converter
- Power supply module 5/3.6 V
- MPU6050 gyroscope module
- ESP8266 Wifi module
- SG90 Microservo
- SR04 Ultrasonic proximity sensor
- 8 contact microswith contact sensors
- Arduino UNO main controller board

The Ardunio controller is driving all the units to allow the robot to collect the sensor data, compute the position relative the initial state, drive the proxcimity sensor direction, drive the robot for a specific direction and speed by feeding back from gyroscpe and speed sensors and comunicate with remote server via wifi sending status and receiving commands.

The wifi module led flashes depending on the comunication state:

- Slow flashing with double blink when acting as access point to "Wheelly" network SSID (no password)
- Slow flashing when acting as network point to configured network SSID
- Medium flashing when client connected and no activity in place
- Fast flashing when client connected and activity in place

