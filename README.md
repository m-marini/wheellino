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

The Ardunio controller is driving all the units to allow the robot to collect the sensor data, compute the position relative the initial state, drive the procimity sensor direction, drive the robot for a specific direction and speed by feeding back from gyroscpe and speed sensors and comunicate with remote server via wifi sending status and receiving commands.

The remote server is Java software running in JVM that allow to drive the robot running different inference engine models (simple manual controller via joystick or state machine base engine).
The software creates a 2D space model from the proximity and contact sensors and implements different basic behaviors that can be composed to build the state machine engine.

The basic behaviors are:
- stop for a while,
- single or random sensor scan with obstacle detection
- move direct to a location with sensor scanning for obstacles
- Find path to location avoiding obstacles (A* algorithm)
- Select sequentially target location from a list
- Select random target location from a list
- Complex behavior to secure the robot in a safe location (far way from obstacles)
- Complex behavior to follow the nearest obstacle at a safe distance
- Wait for robot unblock (robot is in a block state if it cannot move in anyway)
