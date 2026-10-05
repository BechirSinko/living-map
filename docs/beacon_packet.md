# Beacon Message and Signal Design

A beacon is a small battery-powered LoRa node dropped by the Writer. It stores one message and
rebroadcasts it. The message answers the three questions from the challenge:
**what was found**, **where to go**, and **when it was written**.

The format below is implemented identically in `livingmap/beacon.py` (simulation) and
`firmware/include/beacon_msg.h` (ESP32). A test compares the Python output with a frame produced by the
compiled firmware header (`tests/test_beacon.py::test_matches_firmware_golden_frame`).

## Beacon frame (15 bytes on air, little-endian)

Every radio frame is `[type:1][payload]`. For a beacon the payload is the 14-byte `BeaconMsg`.

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `type` | Frame type: 1 = BEACON (2 = LOG, 3 = MISSION) |
| 1 | 1 | `version` | Format version, currently 1 |
| 2 | 1 | `beaconId` | This beacon (1..254) |
| 3 | 1 | `eventType` | 0 waypoint, 1 victim, 2 gas, 3 blocked (reserved) |
| 4 | 1 | `nextHopId` | Next beacon in drop order (0xFF = none) |
| 5 | 2 | `x_cm` | int16, local x in cm (range +-327 m) |
| 7 | 2 | `y_cm` | int16, local y in cm |
| 9 | 2 | `bearingDeg` | int16, 0..359, Writer heading when it dropped the beacon |
| 11 | 4 | `ageAtWriteS` | uint32, seconds since the event was recorded (re-stamped at every broadcast) |

| Question | Fields |
|---|---|
| What was found | `eventType`, and the confidence computed from `ageAtWriteS` |
| Where to go | `x_cm`, `y_cm`, `bearingDeg`, `nextHopId` |
| When it was written | `ageAtWriteS` (the beacon has no clock, only an age counter) |

Hazards need no flag: a beacon with `eventType` = gas is a hazard beacon and is never on a route.

## Other frames

| Frame | Layout | Size | Direction |
|---|---|---|---|
| LOG | `[type=2][BeaconMsg:14][parentId:1]` | 16 B | Writer -> gateway, at the entrance. `parentId` is the beacon it was reached from, so the command post can rebuild the tunnel tree after the Writer backtracks. |
| MISSION | `[type=3][targetEvent:1][n:1][n x (beaconId u8, x_cm i16, y_cm i16)]` | 3 + 5n B, n <= 48 (243 B max) | Gateway -> Executor, at the entrance. The route is planned by the command post. The coordinates let the Executor pick the right branch at a junction. The simulation's mission is 43 B for 8 beacons. |

## Why this design
- **15 bytes** is tiny: at SF7 / 125 kHz the airtime is 46.3 ms, so beacons can broadcast every few seconds without hogging the channel.
- **Centimeter integers** instead of floats: 2 bytes per axis, and encode/decode is identical on the ESP32 and in Python.
- **Local coordinates only.** Beacons never hold GPS, so no beacon needs a GPS fix and the robots stay in their private frame. The gateway translates (`frame_translation.md`).
- **An age counter instead of a timestamp.** The beacon has no clock and no synchronisation step is needed: it counts seconds since it was written (kept across deep sleep) and sends the count with every broadcast.
- **No application CRC.** The SX1276 appends and checks a hardware CRC (`radio.setCRC(true)` on every node); frames that fail it are dropped. This saves 2 bytes. The parsers also reject wrong length, type, version and unknown event types.
- **Next-hop pointers plus coordinates.** Beacons are numbered in drop order and each points to the next one. The route itself comes from the briefing (the command post follows the `parentId` edges), because the next beacon in drop order can be on another branch after a dead end.

## Aging mechanism
A beacon cannot be updated after it is dropped, so trust decays with age. The reader computes

`confidence = exp(-ageAtWriteS / tau)`

| Event | tau | Becomes stale after | Reason |
|---|---|---|---|
| Gas | 600 s | about 12 min | Gas disperses or spreads quickly |
| Victim | 1800 s | about 36 min | A victim stays put but their status changes |
| Blocked path | 1800 s | about 36 min | Debris may shift or be cleared (reserved event) |
| Waypoint | none | never | Geometry barely changes |

Below **0.3** the information is *stale*: the Executor still uses the beacon as a landmark but treats the
hazard as "verify before trusting" and re-checks it with its own sensors. In the simulation the Executor
reaches the beacons about 20 minutes after they were written: the gas beacon is at 0.13 (stale) and the
victim beacon at 0.51 (fresh).

## Signal design (RF)
Identical on every node (writer, executor, beacons, gateway):

| Setting | Value |
|---|---|
| Frequency | 868 MHz band (433 MHz fallback); the legal band in Tunisia is still to be confirmed |
| Spreading factor / bandwidth / coding rate | SF7 / 125 kHz / 4/5 |
| Sync word / preamble | 0x12 (private network) / 8 symbols |
| TX power | 14 dBm (0 dBm when the Writer programs a beacon, so only the beacons in its tube hear it) |

- **Airtime:** 15 B = 46.3 ms, 16 B = 51.5 ms, a 43 B mission = 87 ms. A 1 % duty cycle allows one beacon frame every 4.6 s per node.
- **Rebroadcast:** every 5.5 s plus a random 0-2 s, about 0.7 % duty cycle. With three beacons in range of each other about 3 % of frames overlap, and a missed beacon is heard on its next cycle.
- **No acknowledgements:** beacons are simple one-way broadcasters.
- Initial settings are to be tuned by range tests in Phase 2.

## Reference code
`livingmap/beacon.py` (`Beacon.pack`, `Beacon.unpack`, `aged_confidence`, `is_stale`, `pack_log`, `pack_mission`)
and `firmware/include/beacon_msg.h`.