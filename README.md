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

Documentation in [Wiki](https://github.com/m-marini/wheellino/wiki)
