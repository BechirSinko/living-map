# Implementation Plan

Goal: turn the Phase 1 design and simulation into a working two-robot prototype (Writer + Executor), beacons, and an Outside Network Area (ONA) gateway by the final deadline **01/12/2026**.

## Team split
| Area | Owner | Scope |
|------|-------|-------|
| Software, protocol, command post, docs | Software lead | `livingmap/` (packet, frames, command post, live map), gateway software, simulation, report, manual, repo |
| Embedded, hardware, robots | Embedded lead | `firmware/` (Writer, beacon, gateway, Executor), LoRa, sensors, motors, chassis, beacon dropper |
| Shared | Both | Integration tests, field test, demo video, pitch |

To avoid merge conflicts, Software lead edits `livingmap/` and `docs/`, and the embedded lead edits `firmware/`. Always `git pull --rebase origin main` before pushing.

## Timeline
| Week | Dates | Milestone | Owner | Done when |
|------|-------|-----------|-------|-----------|
| 0 | to 05/10 | Phase 1 submission: repo, simulation, report, failure cases, plan | Both | Submitted before the deadline |
| 1 | 06-12/10 | Order or collect parts; confirm the legal LoRa band in Tunisia; test airtime and range; port `beacon.py` packet to C/C++ | Embedded lead / Software lead | Parts in hand; two ESP32+LoRa nodes exchange a 15-byte beacon frame (16 B log) with the SX1276 hardware CRC |
| 2 | 13-19/10 | Beacon firmware: periodic broadcast, jitter, battery sleep. Gateway firmware: LoRa receive and Wi-Fi/MQTT forward | Embedded lead / Software lead | Gateway forwards a beacon to the command-post software over MQTT |
| 3 | 20-26/10 | Writer base: chassis, motors, odometry/IMU dead reckoning, obstacle sensing, corridor following | Embedded lead | Writer drives a straight and branching test track |
| 3 | 20-26/10 | Command post: live map from MQTT, GPS conversion, mission planner, briefing message | Software lead | Map updates live from gateway test data |
| 4 | 27/10-02/11 | Event sensing (gas sensor, victim detection) and beacon-drop mechanism | Embedded lead | Writer detects both events and drops a beacon, with the packet carrying the right type and position |
| 4 | 27/10-02/11 | Writer exploration logic ported from the simulation; clock sync; log upload at the entrance | Software lead / Embedded lead | Writer explores the test track and uploads its log through the gateway |
| 5 | 03-09/11 | Executor: briefing reception at entrance, beacon reading (RSSI/ID), beacon-to-beacon navigation, re-anchoring, hazard avoidance | Both | Executor follows beacons to the victim marker on the test track |
| 6 | 10-16/11 | Full integration on one mock environment (a small tunnel model with a loop, a gas source, and a victim target) | Both | Complete system chain runs end to end |
| 7 | 17-23/11 | Failure tests (remove a beacon, kill the Writer, corrupt a packet, drop the gateway); fix bugs | Both | Each documented failure case demonstrated or covered |
| 8 | 24/11-01/12 | Documentation: user manual, final report, diagrams, updated repo, pitch (5 min + 2 min Q&A) and demo video | Both | Final submission delivered by 01/12 |

Buffer: weeks 7-8 absorb slips. The riskiest items (LoRa range inside real tunnels, the beacon-drop mechanism, drift on the real chassis) are scheduled early in weeks 1-4.

## Hardware list (to confirm and budget)
- 2 mobile robot chassis with motors, encoders and an IMU (Writer, Executor)
- ESP32 + SX1276 LoRa modules: at least 1 per robot, 1 gateway, 6-10 for beacons
- Gas sensor (e.g. MQ-series) for the hazardous gas event; victim detection via thermal or PIR, or a marker as a stand-in
- Distance/obstacle sensors (ultrasonic or ToF)
- Servo-based beacon dropper on the Writer
- Batteries, chargers, a small router or laptop for the command post

## Risks
| Risk | Impact | Mitigation |
|------|--------|-----------|
| Components arrive late | Delays all firmware work | Order in week 1; start with ESP32 dev boards the team already has |
| LoRa band legality or range problems | Radio link unreliable or non-compliant | Verify the band with the regulator early; test range in a corridor or stairwell |
| Dead-reckoning drift on real hardware is larger than simulated | Beacon positions inaccurate | Add an IMU, and rely on re-anchoring at beacons |
| Beacon-drop mechanism unreliable | Missing beacons | Simple servo-based design; test early; spacing redundancy |
| Integration issues | Late surprises | Weekly integration test from week 2 |
| Embedded lead unavailable | One-person bottleneck | Keep the firmware modular and documented; the simulation stays as a fallback demo |

