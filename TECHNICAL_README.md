# Technical README – Turret Control Project

## Project Status
Prototype demo focused on integration (vision → networking → real-time servo control).  
Contains basic fail-safes. Not production-hardened; security, safety, and performance characterization are work in progress.

---

## Bill of Materials (BOM)

| Item | Description |
|------|-------------|
| ESP32 DevKitC | ESP32-WROOM-32E, SunFounder Ultimate Starter Kit |
| Pan–Tilt Bracket | elechawk dual-axis mount kit, MG996R/S3003 class, compatible with MG995/MG90S form factor |
| 2× MG995 Servo Motors | Metal-gear, 4.8–6.0 V, stall torque ~9.4 kg·cm @ 6V |
| 2-Axis Joystick Module | powered at 3.3 V |
| Momentary Pushbuttons ×2 | With 10 kΩ pulldown resistors, active-high input at 3.3 V |
| Indicator LEDs ×2 | With 220 Ω series resistors, powered from 5 V rail |
| Bench DC Power Supply | HANMATEK HM305M, 0–30 V, 0–5 A; used at 6 V, ≥3 A output for servos |
| Breadboard + jumper wires | Prototyping interconnects, common ground bus across ESP32 + servos + joystick/buttons/LEDs |
| 4 mm Banana to Alligator Clip Leads | PSU → breadboard rail connection |

---

## Circuit Overview

The control system integrates the ESP32 DevKitC with external actuators and inputs via a clean separation of power rails:

- **ESP32** is powered from **USB (5 V)**. The onboard regulator provides **3.3 V**, which is used for the joystick, buttons, and LEDs.
- **Servos (MG995 ×2)** are powered from a dedicated **6 V bench supply** with ≥3 A capacity.  
- **Common Ground (GND_CM)** ties the ESP32 ground and the 6 V supply ground together to ensure a stable reference.
- **Inputs**: joystick on GPIO32/33 (ADC), pushbuttons on GPIO34/35 with 10 kΩ pulldowns.
- **Outputs**: LEDs on GPIO22/23 with 220 Ω resistors, servos on GPIO18/19 (3.3 V PWM).
- **Servo headers (J1/J2)** are 3-pin SIG / +6V_SERVO / GND_CM.

[View schematic (PDF)](assets/ECE230.pdf)
*Figure 1 – Electrical schematic of turret control prototype (Rev A).*



## 3 Pinout & Power
| Function         | Pin         | Notes                           |
|------------------|-------------|---------------------------------|
| SERVO_PAN PWM    | GPIO18      | LEDC 50Hz                       |
| SERVO_TILT PWM   | GPIO19      | LEDC 50Hz                       |
| JOY_X ADC        | ADC1_CH4    | GPIO32                          |
| JOY_Y ADC        | ADC1_CH5    | GPIO33                          |
| RED_BTN	       | GPIO34      | Active-high
| BLUE_BTN	       | GPIO35	     | Active-high
| RED_LED	       | GPIO23	     | Lights in joystick mode
| BLUE_LED	       | GPIO22	     | Lights in WebSocket mode

- **Power**: Servos on **6V rail** with ≥3A headroom. ESP32 on 5V USB. all sharing a common ground (GND_CM) 
- **Grounds must be common** (servo 6V GND tied to ESP32  **GND_CM** ).

---

## Build & Flash
- ESP-IDF: v5.x
```bash
idf.py set-target esp32
idf.py menuconfig   # optional: set Wi-Fi SSID/PASS
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

## 5) Configuration
- `APP_WIFI_SSID` / `APP_WIFI_PASS`  
- `APP_WEBSOCKET_PORT` (default: 8080)  
- `APP_SERVO_MIN_US` / `APP_SERVO_MAX_US` (500–2500 µs)  
- `APP_DEADZONE_ADC` (e.g., 40 / 4095)  
- `APP_FAILOVER_TIMEOUT_MS` (e.g., 100)  

Optional: use `config/app_config.h` for readability.
```
---

##  Configuration
- `APP_WIFI_SSID` / `APP_WIFI_PASS`  
- `APP_WEBSOCKET_PORT` (default: 8080)  
- `APP_SERVO_MIN_US` / `APP_SERVO_MAX_US` (500–2500 µs)  
- `APP_DEADZONE_ADC` (e.g., 40 / 4095)  
- `APP_FAILOVER_TIMEOUT_MS` (e.g., 100)  

Optional: use `config/app_config.h` for readability.

---
##  Modes & State Machine

States: JOYSTICK ↔ WEBSOCKET (+ fallback)

- **JOYSTICK** → Default mode. Joystick drives yaw/pitch. Red LED ON.  
- **WEBSOCKET** → Packets drive yaw/pitch. Blue LED ON.  
- **Mode Select** → RED_BTN → joystick mode, BLUE_BTN → WebSocket mode.  
- **Failover** → if no fresh WS packet within WS_TIMEOUT_MS → fallback to joystick (red LED).  

⚠️ No explicit SAFE neutral state yet (planned).  

## WebSocket Message Schema
```json
{
  "yaw": -45.2,    // [-90, 90]
  "pitch": 12.7    // [-45, 45]
}

```

**Notes:** 
- Values are mapped to servo range:  
  - Yaw: -90 → 0°, +90 → 180°  
  - Pitch: -45 → 180°, +45 → 0°  
- Packets outside valid ranges are clamped.  
- Failover triggered if no packet within timeout.  
---
## Tasking Model

- Implementation runs inside a **single loop in `app_main`** (not split into FreeRTOS tasks).  
- Loop frequency: ~30 Hz (`vTaskDelay(30 ms)`).  
- Responsibilities:  
  - Read joystick ADC values  
  - Apply EMA + deadzone  
  - Handle button presses & mode selection  
  - Update LEDs  
  - Parse WebSocket packets

## Safety & Ethics
Educational **pan-tilt demonstrator for cameras/sensors only**.  
Add a hardware **E-stop** cutting the 5V rail. Follow local laws and safety guidelines.

---

## Media Checklist
- GIF: joystick control sweep  
- GIF: head-tracking via WebSocket  
- YouTube: build + live demo + failover  

---
## Roadmap

- Implement joystick calibration button  
- Add explicit SAFE neutral state on reset/brownout  
- Add LCD mode display (planned in TODO)  
- Latency/jitter instrumentation & plots  
- TLS/auth or MAC allow-list + CRC  
- Slew/jerk limiting, anti-windup  
- PID or model-based control  
- OTA updates + richer telemetry  

---

## Troubleshooting

- **Servos twitch?** → Add bulk cap, better supply, common GND.  
- **ESP32 resets?** → Brownout: stronger 5V supply, shorter wires.  
- **WS connects but no motion?** → Check JSON fields `"yaw"` / `"pitch"`.  
- **Joystick off-center?** → Adjust hardcoded center or implement calibration.  
- **Hums at end-stops?** → Tighten soft limits, add slew limiting.  


