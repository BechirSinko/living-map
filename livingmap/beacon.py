"""Beacon packet: a fixed 16-byte message (what was found, where to go, when it was written).

Layout (big-endian, 16 bytes):

  off size field        notes
  --- ---- ----------   -----------------------------------------------
   0   1   beacon_id    this beacon (1..254; 0 = unused)
   1   1   type_flags   high nibble = event type, low nibble = flags
   2   1   prev_id      previous beacon in the chain (0xFF = none)
   3   1   next_id      next beacon toward the target (0xFF = none)
   4   2   x_cm         int16, local frame, centimeters
   6   2   y_cm         int16, local frame, centimeters
   8   4   timestamp    uint32, Unix seconds when the beacon was written
  12   1   confidence   uint8, 0..255 maps to 0.0..1.0 at write time
  13   1   value        uint8, event magnitude (gas level, victim signal)
  14   2   crc16        CRC-16/CCITT-FALSE over bytes 0..13

Positions are in the Writer's private frame (origin at the entrance).
Beacons never carry GPS: translation is done by the gateway.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum

PACKET_SIZE = 16
NONE_ID = 0xFF
_FMT = ">BBBBhhIBB"  # 14 bytes, then 2 bytes CRC
_BODY_SIZE = struct.calcsize(_FMT)
assert _BODY_SIZE == 14


class EventType(IntEnum):
    WAYPOINT = 0   # chain marker / junction (no event)
    GAS = 1        # hazardous gas
    VICTIM = 2     # victim detected
    BLOCKED = 3    # path blocked (optional third event)


class Flag:
    JUNCTION = 0x1
    DEAD_END = 0x2
    HAZARD_AHEAD = 0x4
    ENTRANCE = 0x8


# Confidence half-life per event type, in seconds.
# Gas moves fast, a victim stays put longer, a waypoint is basically static.
HALF_LIFE_S = {
    EventType.GAS: 600,
    EventType.VICTIM: 3600,
    EventType.BLOCKED: 1800,
    EventType.WAYPOINT: 24 * 3600,
}
STALE_THRESHOLD = 0.25  # below this, treat the info as "verify first"


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). check('123456789') == 0x29B1."""
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class PacketError(ValueError):
    pass


@dataclass
class Beacon:
    beacon_id: int
    event_type: EventType
    x: float                      # meters, local frame
    y: float
    prev_id: int = NONE_ID
    next_id: int = NONE_ID
    timestamp: int = 0            # Unix seconds
    confidence: float = 1.0       # 0..1 at write time
    value: int = 0                # 0..255
    flags: int = 0

    # ---- encoding -------------------------------------------------------
    def pack(self) -> bytes:
        if not 1 <= self.beacon_id <= 254:
            raise PacketError("beacon_id must be 1..254")
        for name, v in (("prev_id", self.prev_id), ("next_id", self.next_id)):
            if not 0 <= v <= 255:
                raise PacketError(f"{name} must be 0..255")
        if not 0 <= int(self.flags) <= 0xF:
            raise PacketError("flags must fit in 4 bits")
        if not 0 <= int(self.event_type) <= 0xF:
            raise PacketError("event_type must fit in 4 bits")
        if not 0 <= self.value <= 255:
            raise PacketError("value must be 0..255")
        x_cm, y_cm = round(self.x * 100), round(self.y * 100)
        if not (-32768 <= x_cm <= 32767 and -32768 <= y_cm <= 32767):
            raise PacketError("position out of range (+-327 m from entrance)")
        if not 0 <= self.timestamp <= 0xFFFFFFFF:
            raise PacketError("timestamp out of range")
        conf = round(max(0.0, min(1.0, self.confidence)) * 255)
        body = struct.pack(
            _FMT, self.beacon_id, (int(self.event_type) << 4) | int(self.flags),
            self.prev_id, self.next_id, x_cm, y_cm, self.timestamp, conf, self.value,
        )
        return body + struct.pack(">H", crc16_ccitt(body))

    @classmethod
    def unpack(cls, data: bytes) -> "Beacon":
        if len(data) != PACKET_SIZE:
            raise PacketError(f"expected {PACKET_SIZE} bytes, got {len(data)}")
        body, (crc,) = data[:_BODY_SIZE], struct.unpack(">H", data[_BODY_SIZE:])
        if crc16_ccitt(body) != crc:
            raise PacketError("CRC mismatch (corrupted packet)")
        bid, tf, prev, nxt, x_cm, y_cm, ts, conf, val = struct.unpack(_FMT, body)
        try:
            et = EventType(tf >> 4)
        except ValueError as e:
            raise PacketError(f"unknown event type {tf >> 4}") from e
        return cls(bid, et, x_cm / 100, y_cm / 100, prev, nxt, ts, conf / 255, val, tf & 0xF)

    # ---- aging ----------------------------------------------------------
    def age_s(self, now: float | None = None) -> float:
        return max(0.0, (time.time() if now is None else now) - self.timestamp)

    def aged_confidence(self, now: float | None = None) -> float:
        """Confidence decays exponentially: c(t) = c0 * 0.5 ** (age / half_life)."""
        half_life = HALF_LIFE_S[self.event_type]
        return self.confidence * 0.5 ** (self.age_s(now) / half_life)

    def is_stale(self, now: float | None = None) -> bool:
        return self.aged_confidence(now) < STALE_THRESHOLD
