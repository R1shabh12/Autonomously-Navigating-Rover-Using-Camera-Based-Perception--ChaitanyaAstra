# Autonomously-Navigating-Rover-Using-Camera-Based-Perception--ChaitanyaAstra

DART Camera-First Autonomous Rover — Reference Implementation
This package is a modular reference implementation matching the DART Task 2 architecture:
Camera → preprocessing/perception → state estimation → risk/planning → safety supervisor
→ differential-drive control → MCU PWM → H-bridge → motors → feedback.
Hardware assumptions
Onboard computer: Raspberry Pi / Jetson-class SBC
Camera: USB/UVC RGB camera
MCU: Arduino-compatible board or ESP32 using Arduino framework
Differential drive with left/right wheel commands
Dual-channel H-bridge with PWM + direction
Quadrature wheel encoders
Optional MPU6050-class I2C IMU (interface can be added)
Optional HC-SR04-style ultrasonic safety sensor
USB serial / UART with newline-delimited JSON
Software
Python: OpenCV, NumPy, PySerial
MCU: Arduino C/C++
Important
The DART report defines the functional architecture but does not specify exact pins, exact motor
controller board, exact camera model, or a production-grade perception model. Therefore hardware
parameters are isolated in config/, and the vision module is intentionally replaceable.
The included vision is a conservative prototype using a green visual goal marker plus a coarse
free-space confidence heuristic. For a real outdoor rover, replace the baseline detector with a
trained terrain/free-space + obstacle model while retaining the state estimation, planner, safety,
controller, and MCU layers.
