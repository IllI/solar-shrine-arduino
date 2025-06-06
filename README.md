# Solar Shrine Arduino Controller

Arduino-based controller for an interactive art installation using ultrasonic sensors and motor control.

## Hardware Components
- 2x HC-SR04 ultrasonic sensors
- L298N motor driver board
- DC motor
- Arduino board
- 9V power supply

## Pin Configuration
### Ultrasonic Sensors
- Sensor 1: TRIG=9, ECHO=10
- Sensor 2: TRIG=5, ECHO=6

### Motor Control (L298N)
- IN3: Pin 4
- IN4: Pin 5
- ENB: Pin 11

## Functionality
The system activates a motor when no hands are detected within range (1-20cm) of both ultrasonic sensors. When hands are detected by both sensors, the motor stops. The system outputs serial data (1/0) for integration with TouchDesigner. 