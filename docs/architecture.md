# Technical Solution and Architecture

## Zones
- **Inside (tunnel):** Writer, beacon chain, Executor. No GPS, no network.
- **Outside Network Area (ONA):** ESP32 + LoRa gateway at the entrance. Receives, translates to GPS, forwards.
- **Command post:** laptop with live map and mission planner.

## Diagram
See the Mermaid diagram in the root `README.md`. Export a PNG to `docs/diagrams/` for the report.

## Component responsibilities
| Component | Responsibility | Tech (proposed) |
|---|---|---|
| Writer | Autonomous exploration, sensing, beacon deployment | ESP32, encoders, IMU, ultrasonic/lidar, gas sensor, PIR/thermal |
| Beacon | Store and broadcast one packet, age it | MCU + LoRa SX1276, battery |
| ONA gateway | Receive, translate, forward, brief | ESP32 + LoRa, Wi-Fi/MQTT |
| Command post | Live map, mission definition | Python + web map |
| Executor | Receive mission, follow beacons, avoid hazards | Same chassis family as Writer, LoRa receiver |

## Key design decisions
1. Memory lives in the beacons, not in the Writer.
2. The gateway does frame translation, robots only use their private frame.
3. Beacons are dumb broadcasters and do not relay to each other.
