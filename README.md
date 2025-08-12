# Turret Control Project

This project is an ESP32-based turret system built using the ESP-IDF framework. It uses servo motors for pan and tilt control, allowing the turret to sweep an area and return to a centered position. This is the foundational layer for an interactive, automated turret system — suitable for future integration with sensors, tracking algorithms, or remote control interfaces.

___

##  Purpose

The primary goal of this project is to build a modular, real-time embedded control system using modern IoT development tools. It simulates a surveillance or targeting turret that can sweep an area and reset, providing the groundwork for more advanced features such as object detection, computer vision integration, or remote (web/mobile) control.

This project was designed as a practical way to:
- Deepen familiarity with **embedded systems programming** using **ESP-IDF**
- Gain hands-on experience with **FreeRTOS task scheduling**
- Understand **servo motor control** using PWM signals
- Develop clean, modular firmware for microcontroller-based systems

## Features

-  **Dual-axis servo control** (pan and tilt)
-  Repeated movement: clockwise sweep followed by recentering
- **GPIO control** for external devices (LEDs, future sensors)
- Real-time scheduling with **FreeRTOS**
- Ready for Wi-Fi integration via ESP32's built-in networking stack


## Future Enhancements
- Ultrasonic or IR distance sensors
- Target tracking via AI/ML (e.g., TinyML / ESP32-CAM + OpenCV)
- Joystick or web-based control (via WebSocket or Bluetooth)
- Safety logic, status LEDs, and override controls

##  Notes for Reviewers & Recruiters
- This project was intentionally built with:
- ESP-IDF and FreeRTOS to reflect industry-grade embedded development (beyond Arduino-level abstraction)
- Emphasis on code clarity, modularity, and real-time control
- Future extensibility: codebase is structured to allow integration of sensors, wireless control, and computer vision
- The repository is self-contained and actively maintained, with room for growth into more advanced robotic control systems.

## Author
Alexander Valdovinos MEna.
