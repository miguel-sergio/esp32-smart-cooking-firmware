# Milestone Plan — ESP32 Smart Cooking Appliance Firmware
**Version:** 1.0  
**Status:** Signed off — development starts at M1  
**References:** SRD_SmartCooking v1.0

---

## How this works

This project is delivered in milestones. Each milestone has a defined set
of deliverables and a clear acceptance criterion — something specific and
observable that confirms the milestone is done before the next one starts.

You review and approve each milestone before I move forward.

---

## Milestone 0 — Scope & Requirements
**Cost:** No charge

**Deliverables:**
- System Requirements Document (SRD) — all requirements defined,
  reviewed, and signed off by you
- This Milestone Plan — signed off by you before development starts

**Acceptance criterion:**
You read the SRD and say "yes, this is exactly what I need."
No open questions before M1 begins.

**Note:** This is the most important milestone. Every hour spent getting
the spec right here saves multiple hours of rework later. No developer
should start coding before this is done — and most do.

---

## Milestone 1 — Proof of Concept
**Deliverable:** Working hardware demo + System Design Document

The fastest path to something visible and running on your hardware.
Validates that all components work correctly together before the full
firmware architecture is built on top of them.

**Deliverables:**
- BME280 reading temperature and humidity in real time, visible on
  serial monitor
- Gear motor spinning under firmware control via DRV8833
- GitHub repository with CI build pipeline active — every push
  automatically compiled and verified from day one
- System Design Document (SDD) — architecture diagrams, FreeRTOS task
  design, state machine, data flows, and full requirements traceability
  matrix linking every design decision to the SRD

**Acceptance criterion:**
Short video or serial monitor output showing sensor readings and motor
responding to commands. CI build badge green. SDD reviewed and approved
before M2 begins.

**Note:** Hardware surprises are common. Finding them here — before the
full architecture is built — is far cheaper than finding them in M3.

---

## Milestone 2 — Core Firmware Architecture
**Deliverable:** Production-grade real-time task structure running on hardware

The full firmware foundation is implemented. All subsystems run as
independent tasks with clean, safe communication between them.

**Deliverables:**
- Four concurrent FreeRTOS tasks running: thermal, motor, control,
  communications — isolated and independently testable
- All inter-task communication via message queues — no shared memory,
  no race conditions
- Watchdog timers active on all tasks — system recovers cleanly if
  any task hangs
- Hardware abstraction layer — swapping a sensor or driver requires
  only a driver-level change, not logic changes
- Motor ramp-up and ramp-down working smoothly via DRV8833
- Static code analysis running in CI on every push

**Acceptance criterion:**
System runs continuously for 30 minutes under load with no crashes,
no watchdog triggers, and clean log output on serial monitor.

---

## Milestone 3 — Cooking Control Logic
**Deliverable:** Complete cooking cycle running end to end

The cooking intelligence of the appliance is implemented: state machine,
PID thermal regulation, motor profiles, and fault protection.

**Deliverables:**
- Full cooking cycle executing automatically: IDLE → PREHEAT →
  COOKING → DONE, with correct transitions as defined in the SRD
- Bang-bang thermal regulation maintaining target temperature within ±1 °C
  of the profile setpoint
- Motor speed profile linked to each cooking phase
- Fault detection active — OVERTEMP, SENSOR_TIMEOUT, ESTOP all
  transition correctly to ERROR state and halt all actuators
- Both cooking profiles stored in NVS, selectable at runtime
- Unit tests covering state machine and thermal control logic, running in CI

**Acceptance criterion:**
Full cooking cycle runs end to end without errors using both profiles.
Fault injection test (BME280 disconnected mid-cycle) triggers ERROR
state correctly and halts all actuators immediately.

---

## Milestone 4 — Connectivity & Remote Control
**Deliverable:** Device monitored and controlled remotely, OTA verified

The device connects to Wi-Fi, publishes live telemetry, accepts remote
commands, and supports field firmware updates.

**Deliverables:**
- Wi-Fi provisioning via BLE (Blufi) — no hardcoded credentials,
  no reflashing required
- Live telemetry published to MQTT at rates defined in the SRD
- All remote commands accepted and processed as defined in the SRD
  (START, STOP, ESTOP, RESET, profile selection)
- OTA firmware update triggered via MQTT, delivered over HTTPS
- Automatic rollback verified: deliberately corrupted firmware image
  reverts to previous working version on next boot

**Acceptance criterion:**
Full cooking cycle started, monitored, and stopped remotely via MQTT.
ESTOP command halts all actuators within 500ms. OTA update applied
successfully. Rollback verified with forced bad image.

---

## Milestone 5 — Delivery
**Deliverable:** Final codebase, documentation, and public repository

**Deliverables:**
- Full code audit: consistent naming, clear comments, no dead code
- All CI checks passing: build, static analysis, unit tests
- README published on GitHub: project overview, hardware setup,
  build instructions, architecture diagrams, engineering decisions
- All release pipeline reports delivered as artifacts (coverage,
  complexity, static analysis, documentation)
- Repository public and ready for your team to maintain

**Acceptance criterion:**
All CI badges green. You confirm all deliverables match the acceptance
criteria defined in the SRD.

---

## Summary

| Milestone | Key deliverable | Gate |
|-----------|-----------------|------|
| M0 — Scope & Requirements | SRD + Milestone Plan signed off | Your approval |
| M1 — Proof of Concept | Hardware demo + SDD approved | Your approval |
| M2 — Core Architecture | FreeRTOS tasks stable 30 min | Automated + your review |
| M3 — Control Logic | Full cycle + fault injection pass | Automated + your review |
| M4 — Connectivity & OTA | Remote control + rollback verified | Automated + your review |
| M5 — Delivery | Clean repo + all reports delivered | Your final sign-off |

---

## Change requests

Any change to agreed scope after M0 sign-off is a change request.
Small clarifications within existing scope are handled at no extra charge.
Changes that affect timeline or cost are agreed in writing before
work begins.

---

*This plan is read alongside the SRD, which defines the detailed acceptance
criteria for each deliverable.*
