# Firmware (PlatformIO)

One project, one environment per program. Shared radio settings and packet formats: `include/beacon_msg.h`
(edit it in one place, all programs pick it up).

| Environment | Program | Board |
|---|---|---|
| beacon1, beacon2, beacon3 | `src/beacon` (ID set in `platformio.ini`) | ESP32 + SX1276 |
| gateway | `src/gateway` | ESP32 + SX1276 |
| receiver | `src/receiver` (Executor listening side) | ESP32 + SX1276 |
| writer | `src/writer` (WRITE + servo drop) | ESP32 + SX1276 + servo |

## Commands

```
pio run -e gateway                          # build
pio run -e beacon2 -t upload                # build and flash
pio run -e beacon2 -t upload --upload-port COM5   # when several boards are plugged in
pio device monitor -b 115200                # serial monitor
```

Before building the gateway, copy `include/secrets.example.h` to `include/secrets.h` (git-ignored) and fill in the Wi-Fi name and password, the MQTT broker and the tunnel entrance GPS. `HEADING_DEG` is the compass bearing of the local +x axis, clockwise from true north.
