# System Requirements Document — ESP32 Smart Cooking Appliance Firmware
**Version:** 1.0  
**Status:** Closed — signed off for development  
**References:** Job posting — automatic cooking appliance

---

## 1. System overview

The system is an automatic cooking appliance controlled by an ESP32
microcontroller. The firmware manages a stirring motor, a temperature
and humidity sensor, and a heating element. The device executes
configurable cooking cycles autonomously and can be monitored and
controlled remotely over Wi-Fi. Wi-Fi credentials are configured via
BLE on first boot. Firmware can be updated over the air.

---

## 2. Functional requirements

### FR-01 — Cooking cycle execution
The firmware shall execute a cooking cycle consisting of four sequential
phases: IDLE, PREHEAT, COOKING, DONE. Transitions between phases are
automatic based on the conditions defined below:

- IDLE → PREHEAT: START command received
- PREHEAT → COOKING: measured temperature reaches the preheat threshold
  defined in the active cooking profile
- COOKING → DONE: cooking duration defined in the active profile has elapsed
- DONE → IDLE: 60 seconds after entering DONE, or on explicit STOP command

### FR-02 — Temperature and humidity monitoring
The firmware shall read temperature and humidity from a BME280 sensor
via I2C at 1 Hz during an active cycle and at 0.1 Hz in IDLE. Readings
shall be validated before use. A reading is invalid if it falls outside
the following ranges:

- Temperature: −40°C to 85°C (BME280 operating range)
- Humidity: 0% to 100% RH

### FR-03 — Thermal regulation
The firmware shall regulate the heating element using bang-bang control with
hysteresis. The relay is turned ON when measured temperature falls more than
1 °C below the target, and OFF when it rises more than 1 °C above the target.
The control loop runs at 1 Hz inside `thermal_task`.

This approach is sufficient for the heating dynamics of this appliance and
requires no tuning. PID control is deferred to a future version (see §7 — Future Work).

### FR-04 — Motor control
The firmware shall control the stirring motor via PWM through the DRV8833
motor driver. Motor behavior per phase:

| Phase | Motor state | Duty cycle |
|-------|-------------|------------|
| IDLE | Stopped | 0% |
| PREHEAT | Stopped | 0% |
| COOKING | Running forward | Profile-defined (default 60%) |
| DONE | Stopped | 0% |
| ERROR | Stopped immediately | 0% |

Motor ramp-up time: 2 seconds from 0% to target duty cycle.
Motor ramp-down time: 1 second from target duty cycle to 0%.
No abrupt starts or stops under any condition except ERROR and ESTOP.

### FR-05 — Fault detection and safe state
The firmware shall monitor for the following fault conditions continuously
during an active cycle. Any fault transitions the system immediately to
ERROR state.

| Fault | Condition |
|-------|-----------|
| OVERTEMP | Measured temperature exceeds profile safety limit |
| SENSOR_TIMEOUT | No valid BME280 reading for more than 3 seconds |
| ESTOP | Emergency stop command received remotely |
| Heater failure *(CR-001)* | Temperature remains more than 1 °C below the cooking target for 2 consecutive minutes at any point during the COOKING phase; the window resets if temperature recovers |

> **CR-001** — *Change request raised 2026-04-26.* Added a heater-failure fault to detect a non-functioning or degraded heating element at any point during the COOKING phase. The system monitors temperature continuously; if it remains more than 1 °C below the cooking target for 2 consecutive minutes, the system transitions to ERROR. A temporary dip followed by recovery resets the consolidation window and does not trigger the fault.

In ERROR state: motor stops immediately, heating element PWM set to 0%,
fault type and timestamp logged via UART, fault published remotely.
Recovery from ERROR requires explicit RESET command — no automatic recovery.

### FR-06 — Cooking profiles
The firmware shall support 2 cooking profiles stored in NVS. Profiles
persist across power cycles and OTA updates. Each profile defines:

| Parameter | Profile 0 (default) | Profile 1 |
|-----------|--------------------|-----------| 
| Name | Standard | Delicate |
| Preheat threshold | 60°C | 45°C |
| Target temperature | 80°C | 60°C |
| Safety limit | 95°C | 75°C |
| Cook duration | 30 min | 20 min |
| Motor duty cycle | 60% | 40% |

Active profile is selectable remotely. Profile parameters are not
user-editable in v1.0 — profiles are fixed in firmware.

### FR-07 — Wi-Fi provisioning
The firmware shall support Wi-Fi credential configuration via BLE using
the ESP-IDF Blufi protocol on first boot (no credentials in NVS).
Credentials are saved to NVS on successful provisioning. BLE stack is
disabled immediately after provisioning completes. On subsequent boots,
the device connects to Wi-Fi directly using stored credentials without
entering BLE provisioning mode. If Wi-Fi connection fails after 3
consecutive attempts, the firmware automatically re-enters BLE provisioning
mode — no physical button required. This allows credential updates whenever
the network changes: the device will fail to connect and fall back to BLE
provisioning on the next power cycle.

### FR-08 — Remote monitoring
The firmware shall publish device telemetry over Wi-Fi using MQTT to a
broker defined at build time via sdkconfig. Telemetry topics and rates:

| Topic | Content | Rate |
|-------|---------|------|
| `cooking/telemetry` | Temperature, humidity, phase, motor state, active profile | 1 Hz during active cycle, 0.1 Hz in IDLE |
| `cooking/fault` | Fault type, timestamp, last valid temperature | On fault event only |

Payload format: JSON. QoS level: 1.

### FR-09 — Remote control
The firmware shall subscribe to MQTT command topics and process the
following commands:

| Topic | Command | Action |
|-------|---------|--------|
| `cooking/cmd` | `START` + profile_id | Start cycle with selected profile |
| `cooking/cmd` | `STOP` | Stop active cycle, return to IDLE |
| `cooking/cmd` | `ESTOP` | Immediate stop, transition to ERROR |
| `cooking/cmd` | `RESET` | Clear ERROR state, return to IDLE |

Commands received during incompatible states (e.g. START while already
COOKING) are rejected and logged. No silent ignoring.

### FR-10 — OTA firmware updates
The firmware shall support OTA updates delivered as binary files over
HTTPS. Update is triggered by a dedicated MQTT command on topic
`cooking/ota` with a URL payload pointing to the firmware binary.

OTA update is deferred if a cooking cycle is active — it queues and
executes after the cycle reaches DONE or ERROR. The device uses ESP-IDF
dual-partition OTA. A new firmware image must call
`esp_ota_mark_app_valid_cancel_rollback()` after a successful boot
self-check, or the bootloader reverts to the previous partition
automatically on next restart.

---

## 3. Non-functional requirements

| ID | Requirement |
|----|-------------|
| NFR-01 | Watchdog timers active on all FreeRTOS tasks. Any task that fails to check in within its timeout triggers a controlled system restart, logged via UART. |
| NFR-02 | Control tasks (thermal, motor, control) run on Core 1. Communications task runs on Core 0. Control loop execution is never blocked by network activity. |
| NFR-03 | All hardware-specific code sits behind a HAL. Replacing BME280 or DRV8833 requires only driver-layer changes — zero changes to control or application logic. ESP-IDF native APIs only. No Arduino framework. |
| NFR-04 | Structured UART logging using ESP-IDF log levels (ERROR, WARN, INFO, DEBUG). Every state transition, fault event, and command received is logged with timestamp. |
| NFR-05 | OTA dual-partition scheme. New firmware marked valid only after successful boot self-check. Invalid firmware triggers automatic rollback. |
| NFR-06 | CI pipeline on GitHub Actions: build verification, static analysis (cppcheck + clang-tidy), and Unity unit tests (state machine + thermal control) on every push. Zero static analysis errors on main branch. |

---

## 4. Hardware

| Component | Selection | Interface |
|-----------|-----------|-----------|
| Microcontroller | ESP32-WROOM-32 | — |
| Temperature & humidity sensor | BME280 | I2C (address 0x76) |
| Motor driver | DRV8833 | PWM (2 channels) |
| Stirring motor | Gear motor 6V 200RPM | PWM via DRV8833 |
| Heating element interface | Relay (time-proportioning, 2 s period) | GPIO output |
| Connectivity | Wi-Fi 2.4GHz + BLE 4.2 | Onboard ESP32 |
| Power supply | 5V USB for ESP32, 6V for motor | — |

---

## 5. Scope boundary

| In scope | Out of scope |
|----------|-------------|
| ESP32 firmware (ESP-IDF, C/C++) | Mobile app or web UI |
| BME280 driver (I2C) | Cloud backend infrastructure |
| DRV8833 motor driver (PWM) | PCB design or hardware manufacturing |
| Heating element PWM control | Safety certifications (CE, UL, etc.) |
| Cooking state machine | Voice control or third-party integrations |
| Bang-bang thermal regulation | PID thermal regulation (future work) |
| BLE Wi-Fi provisioning (Blufi) | User authentication |
| MQTT telemetry and control | Physical enclosure or mechanical design |
| OTA with automatic rollback | Sensor calibration beyond range validation |
| NVS cooking profiles (2 profiles) | — |
| CI pipeline (build, static analysis, unit tests) | — |

---

## 6. Acceptance criteria

The firmware is considered complete and ready for delivery when all of
the following are met:

1. Full cooking cycle (IDLE → PREHEAT → COOKING → DONE) executes on
   hardware using both profiles without errors
2. Fault injection test: BME280 disconnected mid-cycle triggers OVERTEMP
   or SENSOR_TIMEOUT → ERROR state, motor stops, heater off
3. ESTOP command received via MQTT halts all actuators within 500ms
4. Device provisions Wi-Fi via BLE on first boot and connects to MQTT
   broker successfully
5. All telemetry fields publish at the specified rates during an active cycle
6. OTA update applied successfully from a remote URL
7. Rollback triggered and verified with a deliberately corrupted firmware image
8. All CI checks pass (build, static analysis, unit tests) on final commit
9. README documents hardware setup, architecture, and engineering decisions

---

## 7. Future work

The following items are explicitly out of scope for v1.0 and deferred to future versions:

| Item | Rationale |
|------|-----------|
| PID thermal regulation | Requires characterisation of the heating element. Bang-bang control with ±1 °C hysteresis is sufficient for v1.0. |
| MOTOR_STALL detection | Requires a tachometer or current sensor not present in the v1.0 hardware. |
| Live PID gain update via MQTT | No PID in v1.0. |

---

*Signed off for development. Changes to this document after M0 are treated
as change requests and may affect timeline and cost.*
