# Firmware (Phase 2)

Embedded code for the physical prototype. Each folder holds one device:

- `writer/` Writer robot (exploration, sensing, beacon dropper)
- `beacon/` LoRa beacon (broadcast, aging)
- `gateway/` Outside Network Area gateway (LoRa receive, Wi-Fi/MQTT forward)
- `executor/` Executor robot (briefing, beacon-guided navigation)
