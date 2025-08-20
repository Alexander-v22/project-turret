# Turret Control Project

![C](https://img.shields.io/badge/Language-C-blue)
![ESP-IDF](https://img.shields.io/badge/Framework-ESP--IDF-green)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange)
![WiFi](https://img.shields.io/badge/Connectivity-WiFi-lightgrey)
![WebSocket](https://img.shields.io/badge/Protocol-WebSocket-purple)


An ESP32-based dual-axis turret system built on the ESP-IDF framework and FreeRTOS.  
The system uses servo motors for pan and tilt control, supports manual input via joystick, and provides a Wi-Fi + WebSocket interface for remote real-time control. This project demonstrates embedded systems design across hardware control, signal processing, networking, and system integration.

---

## Demo

| Turret Sweep | Joystick Control | WebSocket Control |
|--------------|-----------------|-------------------|
| (Insert GIF here) | (Insert GIF here) | (Insert GIF here) |

Video Walkthrough: [YouTube Demo](https://youtu.be/your-demo-link)

---

## Purpose

The primary goal of this project is to build a modular, real-time embedded control system on the ESP32 platform. It simulates a pan-tilt turret for surveillance or targeting, laying the foundation for more advanced features such as:

- Object detection and computer vision (e.g., ESP32-CAM + OpenCV/TinyML)
- Sensor-based automation (IR, ultrasonic, or PIR tracking)
- Interactive remote control via web, mobile, or joystick

This project was designed to:

- Deepen familiarity with embedded systems programming using ESP-IDF
- Gain hands-on experience with FreeRTOS task scheduling
- Explore servo control via PWM (LEDC hardware peripheral)
- Implement real-time networking with WebSockets
- Develop a scalable, extensible firmware codebase

---

## Features
The turret is designed as a modular embedded platform with multiple layers of control. 
At its core, it provides precise servo actuation for dual-axis movement. Input can come 
from either physical hardware (a joystick) or a wireless client over Wi-Fi. 

Key features include:
- Dual-axis servo control (pan + tilt)
- Joystick input (2-axis potentiometers mapped via ADC with 12-bit resolution)
- Wi-Fi station mode for wireless connectivity
- WebSocket server for low-latency, bidirectional control messages
- PWM generation using LEDC peripheral (16-bit Q16 fixed-point format, 50 Hz)
- Servo stability helpers: quantization handling, soft deadband, optional 1-Euro filter
- GPIO expansion for LEDs, sensors, and future modules
- FreeRTOS tasking model for responsive, non-blocking control

---
### Control Modes and Indicators
The turret supports two control modes selectable via on‑board buttons:

- Joystick Mode: selected by the RED button; RED LED on.
- WebSocket Mode: selected by the BLUE button; BLUE LED on.

If no WebSocket message is received for 2000 ms, the system automatically falls back to Joystick Mode to ensure local control remains responsive.

## System Architecture

```mermaid
flowchart TD
    Joystick[Joystick Input (ADC)] --> ADC_Map[ADC Mapping → Servo Angle]
    Web[WebSocket Control (Yaw/Pitch)] --> JSON_Parser[Minimal JSON Parser]
    ADC_Map --> PWM[LEDC PWM Generator]
    JSON_Parser --> PWM
    PWM --> Servos[Pan-Tilt Servo Motors]
```
---
## Technical Highlights

### Servo Control (PWM via LEDC)
- Frequency: **50 Hz** (20 ms period)  
- Pulse width range: **500–2500 µs → 0–180°**  
- Q16 fixed-point duty cycle resolution (0–65535 counts)  
- Conversion formula: DutyCounts = (Pulse_us / Period_us) * 65536
- Practical angle range: **0–175°** (clamped in firmware to avoid end‑stop jamming)


### Joystick Input
- 2-axis potentiometer joystick (X = yaw, Y = pitch)  
- ADC configuration: **12-bit resolution (0–4095)** with **11 dB attenuation (~2.45V range)**  
- Mapping: Raw ADC → Servo Angle (0–180°) → Duty Counts
- X (pan) mapping is inverted (0–4095 → 175–0) to match physical orientation
- Y (tilt) mapping is normal (0–4095 → 0–175)




### Wi-Fi & Networking
- Mode: **Station (STA)**  
- Initialization flow:  
nvs_flash_init → esp_netif_init → esp_wifi_init → esp_wifi_set_mode → esp_wifi_start → esp_wifi_connect


- Handles WPA2 authentication + DHCP assignment  

### WebSocket Server
- Built on top of ESP-IDF HTTP server  
- URI endpoint: `/ws` with `is_websocket = true`  
- Two-step frame handling (`httpd_ws_recv_frame` probe + full read)  
- Sends/receives control packets (Yaw, Pitch) in near real time  
- Minimal JSON-style parsing for performance  

---

## Potential Future Enhancements
- Integrate ESP32-CAM for computer vision tracking  
- Add autonomous target tracking (AI/ML)  
- On-device filtering (1-Euro filter on ESP32 instead of frontend)  
- WebSocket authentication & encryption  
- Sensor-based auto-scan (ultrasonic/IR)  
- Safety overrides + calibration routines  

---

## Notes for Reviewers & Recruiters
This project was intentionally built with:  
- **ESP-IDF + FreeRTOS** for professional-grade embedded development (beyond Arduino-level abstraction)  
- **Hardware + software integration**, from analog input and PWM to Wi-Fi networking  
- Emphasis on **modularity, clarity, and extensibility**  

Demonstrated skills in:  
- Embedded C / FreeRTOS task management  
- Real-time signal processing and control loops  
- Networking protocols (HTTP, WebSocket)  
- Robotics and IoT system design  

---

## Author
**Alexander Valdovinos Mena**