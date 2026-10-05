"""Beacon packets, byte-for-byte compatible with firmware/include/beacon_msg.h.

Wire frame = [type:1][BeaconMsg:14], little-endian, packed (15 B on air):

  off size field        notes
  --- ---- -----------  ------------------------------------------------
   0   1   type         PKT_BEACON = 1 (LOG = 2, MISSION = 3)
   1   1   version      1
   2   1   beaconId     1..254
   3   1   eventType    0 none/waypoint, 1 victim, 2 gas, 3 blocked
   4   1   nextHopId    next beacon in drop order, 0xFF = none
   5   2   x_cm         int16, private frame, cm
   7   2   y_cm         int16
   9   2   bearingDeg   int16, 0..359, Writer heading when the beacon was dropped
  11   4   ageAtWriteS  uint32, seconds since the event was recorded

LOG     = [type=2][BeaconMsg:14][parentId:1]                 (16 B)
MISSION = [type=3][targetEvent:1][n:1][n x (beaconId u8, x_cm i16, y_cm i16)]   (3 + 5n B, n <= 48)

There is no application CRC: radio.setCRC(true) on every node, bad frames are dropped by the SX1276.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum

PKT_BEACON, PKT_LOG, PKT_MISSION = 1, 2, 3
VERSION = 1
PACKET_SIZE = 15      # [type][BeaconMsg:14]
LOG_SIZE = 16         # [type][BeaconMsg:14][parentId]
NONE_ID = 0xFF
MAX_ROUTE = 48        # 3 + 5*48 = 243 B, under the SX1276 255 B FIFO
_FMT = "<BBBBBhhhI"
assert struct.calcsize(_FMT) == PACKET_SIZE


class EventType(IntEnum):
    NONE = 0      # waypoint, no event
    VICTIM = 1
    GAS = 2
    BLOCKED = 3   # reserved (not used by the simulation)


# One rule for the whole project: confidence = exp(-age / tau). Waypoints do not decay.
TAU_S = {EventType.GAS: 600.0, EventType.VICTIM: 1800.0, EventType.BLOCKED: 1800.0}
STALE_THRESHOLD = 0.3   # below this: verify before trusting


def confidence(event_type: int, age_s: float) -> float:
    tau = TAU_S.get(EventType(event_type))
    return 1.0 if tau is None else math.exp(-max(0.0, age_s) / tau)


class PacketError(ValueError):
    pass


@dataclass
class Beacon:
    beacon_id: int
    event_type: EventType
    x: float                      # metres, private frame
    y: float
    next_hop: int = NONE_ID
    bearing_deg: int = 0          # Writer heading at drop time
    age_s: int = 0                # seconds since written

    def _pack(self, pkt_type: int) -> bytes:
        if not 1 <= self.beacon_id <= 254:
            raise PacketError("beacon_id must be 1..254")
        if not 0 <= self.next_hop <= 255:
            raise PacketError("next_hop must be 0..255")
        if not 0 <= int(self.event_type) <= 3:
            raise PacketError("event_type must be 0..3")
        if not 0 <= self.bearing_deg <= 359:
            raise PacketError("bearing_deg must be 0..359")
        if not 0 <= self.age_s <= 0xFFFFFFFF:
            raise PacketError("age_s out of range")
        x_cm, y_cm = round(self.x * 100), round(self.y * 100)
        if not (-32768 <= x_cm <= 32767 and -32768 <= y_cm <= 32767):
            raise PacketError("position out of range (+-327 m from entrance)")
        return struct.pack(_FMT, pkt_type, VERSION, self.beacon_id, int(self.event_type),
                           self.next_hop, x_cm, y_cm, self.bearing_deg, int(self.age_s))

    def pack(self) -> bytes:
        return self._pack(PKT_BEACON)

    @classmethod
    def _unpack(cls, data: bytes, want_type: int) -> "Beacon":
        if len(data) != PACKET_SIZE:
            raise PacketError(f"expected {PACKET_SIZE} bytes, got {len(data)}")
        t, ver, bid, et, nxt, x_cm, y_cm, brg, age = struct.unpack(_FMT, data)
        if t != want_type:
            raise PacketError(f"unexpected packet type {t}")
        if ver != VERSION:
            raise PacketError(f"unsupported version {ver}")
        if not 1 <= bid <= 254:
            raise PacketError("bad beacon id")
        try:
            ev = EventType(et)
        except ValueError as e:
            raise PacketError(f"unknown event type {et}") from e
        return cls(bid, ev, x_cm / 100, y_cm / 100, nxt, brg, age)

    @classmethod
    def unpack(cls, data: bytes) -> "Beacon":
        return cls._unpack(data, PKT_BEACON)

    def aged_confidence(self) -> float:
        return confidence(self.event_type, self.age_s)

    def is_stale(self) -> bool:
        return self.aged_confidence() < STALE_THRESHOLD


# ---- LOG: Writer -> gateway -------------------------------------------------
def pack_log(b: Beacon, parent_id: int) -> bytes:
    return b._pack(PKT_LOG) + bytes([parent_id])


def unpack_log(data: bytes) -> tuple[Beacon, int]:
    if len(data) != LOG_SIZE:
        raise PacketError(f"expected {LOG_SIZE} bytes, got {len(data)}")
    return Beacon._unpack(data[:PACKET_SIZE], PKT_LOG), data[PACKET_SIZE]


# ---- MISSION: gateway -> Executor (variable length) ----------------------------
def pack_mission(target_event: int, route: list[tuple[int, float, float]]) -> bytes:
    if not 1 <= len(route) <= MAX_ROUTE:
        raise PacketError(f"route must have 1..{MAX_ROUTE} entries")
    out = struct.pack("<BBB", PKT_MISSION, int(target_event), len(route))
    for bid, x, y in route:
        out += struct.pack("<Bhh", bid, round(x * 100), round(y * 100))
    return out


def unpack_mission(data: bytes) -> tuple[int, list[tuple[int, float, float]]]:
    if len(data) < 3 or data[0] != PKT_MISSION:
        raise PacketError("not a mission packet")
    _, target, n = struct.unpack("<BBB", data[:3])
    if len(data) != 3 + 5 * n:
        raise PacketError("mission length mismatch")
    route = [(bid, x / 100, y / 100)
             for bid, x, y in (struct.unpack("<Bhh", data[3 + 5 * i:8 + 5 * i]) for i in range(n))]
    return target, route
