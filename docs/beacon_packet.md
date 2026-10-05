# Beacon Message and Signal Design

A beacon is a small battery-powered LoRa node dropped by the Writer. It stores one packet and
rebroadcasts it. The packet answers the three questions from the challenge:
**what was found**, **where to go**, and **when it was written**.

## Packet layout (16 bytes, big-endian)

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | `beacon_id` | This beacon (1..254) |
| 1 | 1 | `type_flags` | High nibble: event type. Low nibble: flags |
| 2 | 1 | `prev_id` | Previous beacon in the chain (0xFF = none) |
| 3 | 1 | `next_id` | Next beacon toward the target (0xFF = none) |
| 4 | 2 | `x_cm` | int16, local x in cm (range +-327 m) |
| 6 | 2 | `y_cm` | int16, local y in cm |
| 8 | 4 | `timestamp` | uint32 Unix seconds, when written |
| 12 | 1 | `confidence` | uint8, 0..255 maps to 0.0..1.0 |
| 13 | 1 | `value` | uint8 event magnitude (gas level, victim signal) |
| 14 | 2 | `crc16` | CRC-16/CCITT-FALSE over bytes 0..13 |

| Question | Fields |
|---|---|
| What was found | `type_flags`, `value`, `confidence` |
| Where to go | `x_cm`, `y_cm`, `prev_id`, `next_id` |
| When it was written | `timestamp` |

**Event types:** 0 waypoint, 1 gas, 2 victim, 3 blocked.
**Flags:** 0x1 junction, 0x2 dead end, 0x4 hazard ahead, 0x8 entrance.

## Why this design
- **16 bytes** is tiny: at LoRa SF7/125 kHz the airtime is roughly 50 ms (with the LoRa header), so beacons can broadcast every few seconds without hogging the channel. Verify the airtime with a LoRa calculator once the radio settings are chosen.
- **Centimeter integers** instead of floats: 2 bytes per axis, and encode/decode is identical on ESP32 and Python.
- **Local coordinates only.** Beacons never hold GPS, so no beacon needs a GPS fix and the robots stay in their private frame. The gateway translates.
- **Chain pointers** (`prev_id`/`next_id`) let the Executor follow the trail even if one beacon is missed, since neighbours still point past it.
- **CRC-16** rejects corrupted packets (all single-bit errors are detected, tested in `tests/test_beacon.py`).

## Aging mechanism
The reader computes the age from `timestamp` and decays the confidence exponentially:

`confidence(now) = confidence0 * 0.5 ^ ((now - timestamp) / half_life)`

| Event | Half-life | Reason |
|---|---|---|
| Gas | 10 min | Gas disperses or spreads quickly |
| Blocked path | 30 min | Debris may shift or be cleared |
| Victim | 1 h | A victim stays put but their status changes |
| Waypoint | 24 h | Geometry barely changes |

If the aged confidence drops below **0.25** the info is *stale*: the Executor still uses the beacon as a
landmark but treats the hazard as "verify before trusting" and re-checks it with its own sensors.

Timestamp note: the Writer's clock is set at the entrance from the gateway before it enters, so all devices share
the same time base. Without it, ages would be meaningless.

## Signal design (RF)
- LoRa, 433 or 868 MHz (confirm the allowed band in Tunisia). Initial settings: SF7, BW 125 kHz, CR 4/5, to be tuned by range tests in Phase 2.
- Each beacon broadcasts every ~5 s with a small random jitter so neighbouring beacons do not collide repeatedly.
- No acknowledgements: beacons are simple one-way broadcasters.

## Reference code
`livingmap/beacon.py` (`Beacon.pack`, `Beacon.unpack`, `aged_confidence`, `is_stale`).
Port the same layout to C/C++ for the firmware.
