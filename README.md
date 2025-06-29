Wheelly is a wheels robot.

The robot consits of

- Chassis with two lateral wheels driven by 2 x DC motors and a free wheel.
- 2 x IR speed sensors for the wheels
- Motor controller board L298N
- 12V DC rechargable Li battery
- Step down DC-DC 12V-5V converter
- Power supply module 3.3V
- MPU6050 gyroscope module
- ESP32 Vroom main controller board
- SG90 Microservo
- SR04 Ultrasonic proximity sensor
- 6 x microswitch contact sensors
- LCD Display

The ESP32 controller is driving all the units to allow the robot to collect the sensor data, compute the position relative the initial state, drive the proxcimity sensor direction, drive the robot for a specific direction and speed by feeding back from gyroscpe and speed sensors and comunicate with remote server via wifi sending status and receiving commands.

Documentation in [Wiki](https://github.com/m-marini/wheellino/wiki)


qr 1725366947643 A 320 240 63.6 88.9 182.5 100.6 179.3 226.0 58.0 237.0
