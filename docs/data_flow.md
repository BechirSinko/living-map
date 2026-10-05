# Data Flow

1. Writer explores using dead reckoning and frontier-based exploration.
2. Event detected (gas or victim): drop a beacon on a new event, at each junction, and every N meters.
3. Beacon deposited with a compact packet (see `beacon_packet.md`).
4. Beacon broadcasts repeatedly over LoRa; confidence decays with age.
5. Writer returns to the entrance (or low battery) and uploads its log to the gateway.
6. Gateway translates local coordinates to GPS (see `frame_translation.md`).
7. Gateway forwards to the command post, which updates the live map.
8. Command post sends a mission back through the gateway.
9. Executor is briefed at the entrance, then enters.
10. Executor follows beacons in order, avoids hazards, corrects drift at each beacon.
