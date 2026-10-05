# Firmware (PlatformIO)

One project, one environment per program. Shared radio settings and packet formats: `include/beacon_msg.h`
(edit it in one place, all programs pick it up).

| Environment | Program | Board |
|---|---|---|
| beacon1, beacon2, beacon3 | `src/beacon` (ID set in `platformio.ini`) | ESP32 + SX1276 |
| gateway | `src/gateway` | ESP32 + SX1276 |
| receiver | `src/receiver` (Executor listening side) | ESP32 + SX1276 |
| writer | `src/writer` (drop + servo) | ESP32 + SX1276 + servo |

## Wire format

All little-endian. No application CRC — the SX1276 hardware CRC covers each frame and bad frames are dropped.

| Frame   | Layout                                       | Size      |
|---------|----------------------------------------------|-----------|
| BEACON  | `[type:1][BeaconMsg:14]`                     | 15 B      |
| LOG     | `[type:1][BeaconMsg:14][parentId:1]`         | 16 B      |
| MISSION | `[type:1][targetEvent:1][n:1][entry:5]*n`    | 3 + 5n B  |

Packet types: `PKT_BEACON = 1`, `PKT_LOG = 2`, `PKT_MISSION = 3`.

Event values: `EVT_NONE = 0`, `EVT_VICTIM = 1`, `EVT_GAS = 2`, `EVT_BLOCKED = 3`.

`BeaconMsg` carries `ageAtWriteS` (seconds since the fact was written), not a timestamp. The beacon re-stamps it at every transmit.

Confidence is computed on the receiver: `confidence = exp(-age / tau)` with
`tau = 600 s` for gas, `1800 s` for victim, `1800 s` for blocked. Waypoints (`EVT_NONE`) do not decay.
Confidence below `0.3` is treated as stale.

`MISSION` frame: `n <= 48` entries, each `{ beaconId:u8, x_cm:i16, y_cm:i16 }`.

## Commands
pio run -e gateway # build
pio run -e beacon2 -t upload # build and flash
pio run -e beacon2 -t upload --upload-port COM5 # when several boards are plugged in
pio device monitor -b 115200 # serial monitor

////

Before flashing the gateway, copy `include/secrets.example.h` to `include/secrets.h` and fill in the
Wi-Fi name and password, the MQTT broker and the tunnel entrance GPS. `include/secrets.h` is gitignored.