"""Beacon packet: compact message (what was found, where to go, when written).

TODO (Deliverable 2): fixed ~16-byte layout, encode/decode, CRC, aging.
"""
from dataclasses import dataclass


@dataclass
class Beacon:
    beacon_id: int
    event_type: int      # 0=waypoint, 1=gas, 2=victim
    x: float             # local frame, meters
    y: float
    prev_id: int
    next_id: int
    timestamp: int       # seconds since mission start
    confidence: float    # 0..1, decays with age
