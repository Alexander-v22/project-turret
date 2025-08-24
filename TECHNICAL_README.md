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

---

## 3 Pinout & Power
| Function         | Pin         | Notes                           |
|------------------|-------------|---------------------------------|
| SERVO_PAN PWM    | GPIO18      | LEDC 50Hz                       |
| SERVO_TILT PWM   | GPIO19      | LEDC 50Hz                       |
| JOY_X ADC        | ADC1_CH4    | GPIO32                          |
| JOY_Y ADC        | ADC1_CH5    | GPIO33                          |
| RED_BTN	       | GPIO34      | Active-high                     |
| BLUE_BTN	       | GPIO35	     | Active-high                     |
| RED_LED	       | GPIO23	     | Lights in joystick mode         |
| BLUE_LED	       | GPIO22	     | Lights in WebSocket mode        |

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
## Modes & State Machine

The system runs as a two-mode state machine with automatic failover.  
Mode selection is handled via hardware buttons, and LEDs provide a clear indication of the active state:

- **JOYSTICK mode**  
  - Default at power-on reset or when the **RED button** is pressed.  
  - Joystick directly drives yaw and pitch.  
  - **Red LED ON**.

- **WEBSOCKET mode**  
  - Entered when the **BLUE button** is pressed while a WebSocket connection is active.  
  - Remote yaw/pitch values (from packets) drive the servos.  
  - Exits if the **RED button** is pressed or if no valid WebSocket packet is received within the configured timeout (`APP_FAILOVER_TIMEOUT_MS`, default 3000 ms or 3 s).  
  - **Blue LED ON**.

- **FAILOVER (safety fallback)**  
  - Engaged automatically when WebSocket data is lost or stale.  
  - Control reverts to the joystick until either new WebSocket packets arrive (→ WebSocket mode) or the RED button is pressed (→ Joystick mode).  
  - **Red LED ON** (same as joystick mode, indicating safe fallback).

**Notes**  
- Packets outside valid ranges are clamped before mapping to servo angles.  
- A dedicated **SAFE neutral state** is not yet implemented (planned in roadmap).  

---
## System Requirements

The following requirements were defined for the Rev A prototype.  
Verification methods are indicated (Test = bench test, Inspect = code/schematic review, Measure = scope/log capture).

| Req ID | Requirement | Verification Method |
|--------|-------------|----------------------|
| R-01   | The system shall update servo positions with ≤ 50 ms end-to-end latency from input command (joystick or WebSocket). | Measure |
| R-02   | The system shall revert to joystick control within ≤ 100 ms of WebSocket packet loss. | Test |
| R-03   | The servos shall not exceed 0–180° of travel. | Inspect/Test |
| R-04   | The system shall operate continuously for ≥ 1 hour without brownout/reset. | Soak Test |
| R-05   | The system shall indicate active mode via LED (red=joystick, blue=WebSocket). | Inspect/Test |
| R-06   | The firmware shall clamp invalid yaw/pitch inputs to valid range. | Inspect/Test |

---
## WebSocket Message Schema (ICD)

- Encoding: JSON (UTF-8)
- Update Rate: 30 Hz nominal (tolerates ≥ 10 Hz)
- Byte Order: Network order (big-endian)
- Fields:

| Field   | Type     | Units   | Range       | Notes |
|---------|----------|---------|-------------|-------|
| yaw     | float32  | degrees | -90 … +90   | IEEE 754, relative to forward |
| pitch   | float32  | degrees | -45 … +45   | IEEE 754, relative to level |

**Message example:**
```json
{ "yaw": -45.2, "pitch": 12.7 }
```
**Behavior and constraints:**
- Packets outside valid ranges are clamped to the nearest limit.
- If no valid packet is received within APP_FAILOVER_TIMEOUT_MS. 
- Expected update rate: ~30 Hz. Lower rates are tolerated, but responsiveness decreases.
- Packets are assumed to be independent; no sequence numbers or timestamps are currently implemented (planned in roadmap).

**Notes:** 
- Values are mapped to servo range:  
  - Yaw: -90 → 0°, +90 → 180°  
  - Pitch: -45 → 180°, +45 → 0°  
- Packets outside valid ranges are clamped.  
- Failover triggered if no packet within timeout.  
---
## Tasking Model & Timing

The current firmware implements a single control loop within `app_main()`.  
This design favors simplicity over concurrency; no explicit FreeRTOS tasks are used yet.

**Loop characteristics**
- Frequency: ~30 Hz (`vTaskDelay(30 ms)`)
- Execution model: sequential polling of all inputs and outputs
- CPU usage: <10% at this rate (approx., unmeasured)
- Latency budget: joystick/WebSocket input → servo output ≈ one loop cycle (≤30 ms typical)

**Responsibilities per cycle**
1. Read joystick ADC values (GPIO32/33).  
2. Apply filtering (exponential moving average) and deadzone.  
3. Sample button states (GPIO34/35) and handle mode selection.  
4. Update LED indicators (GPIO22/23).  
5. Poll WebSocket for new packets; parse and map yaw/pitch.  
6. Enforce range clamping and write PWM signals to servos (GPIO18/19).

**Limitations (Rev A)**
- No task prioritization or preemption — all logic shares one loop.  
- No explicit WCET (worst-case execution time) analysis performed.  
- Networking and control share the same loop; packet parsing delays could affect servo updates.  
- No watchdog or timing instrumentation yet.

**Timing Budget**

| Stage                   | Budget (ms) | Notes |
|--------------------------|-------------|-------|
| Joystick ADC read/filter | 2           | EMA applied |
| Button/LED update        | 1           | GPIO operations |
| WebSocket packet parse   | 5           | JSON decode |
| Servo PWM update         | 1           | LEDC duty cycle |
| Loop overhead            | 2           | Idle/housekeeping |

**Total per cycle:** ≤ 30 ms (33 Hz max rate)  
**Measured CPU usage:** <10% (approx., uninstrumented)

---
## Safety & Ethics

- **Failover:** Reverts to joystick if WebSocket stale > 3000 ms
- **Clamping:** Servo commands clamped to safe range
- **Startup posture:** Not implemented (future work)
- **Watchdog timer:** Not implemented (future work)
- **Servo stall/brownout:** Not monitored (future work)

Planned improvements (Rev B):
- Hardware watchdog reset
- Configurable safe neutral position
- Current/temperature monitoring for servos 

**Ethics**
- Inspired by aerospace head-tracking concepts, but not affiliated with any proprietary systems.  
- Developed at Duke University (ECE/CS) as an academic exercise in embedded control.  
- Intended only for research, learning, and robotics demonstrations.

---

## Troubleshooting & Known Issues

**Common issues**
- **Servos twitch or jitter** → Ensure servo rail is on a stable 6 V supply with ≥3 A capacity. Verify all grounds are tied together (GND_CM).  
- **ESP32 resets** → Likely brownout from voltage dips; shorten jumper wires and confirm USB cable quality.  
- **WebSocket connects but no motion** → Check that packets contain `yaw` and `pitch` fields in JSON.  
- **Joystick off-center** → Adjust hardcoded center values in firmware (no calibration button implemented).  
- **Servos hum at end-stops** → Tighten software limits or adjust deadzone to prevent saturation.

**Known limitations**
- No explicit neutral “SAFE” state on reset.  
- Single-loop implementation (no FreeRTOS task separation).  
- No TLS/authentication on WebSocket server.  
- No timing instrumentation or performance measurements captured.  

## Verification & Test Plan (Outline)

- **Latency measurement:** Oscilloscope trace (joystick input → servo PWM output)
- **Failover test:** Drop WebSocket packets, verify joystick control resumes ≤ 100 ms
- **Brownout test:** Reduce servo rail voltage, observe ESP32 stability
- **Packet fuzzing:** Send malformed/NaN inputs, verify clamping
- **Soak test:** Continuous operation ≥ 8 hours

## Revision History
| Rev | Date       | Description                  |
|-----|------------|------------------------------|
| A   | Aug 2025   | Initial prototype integration (vision → ESP32 → servos) |


*These limitations are inherent to the Rev A prototype and are not scheduled for further development as of now.*
