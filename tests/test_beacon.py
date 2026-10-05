import math
import pytest

from livingmap.beacon import (NONE_ID, PACKET_SIZE, PKT_BEACON, Beacon, EventType, PacketError,
                              confidence, pack_log, pack_mission, unpack_log, unpack_mission)


def sample(**kw):
    d = dict(beacon_id=7, event_type=EventType.GAS, x=12.34, y=-5.67, next_hop=8,
             bearing_deg=270, age_s=120)
    d.update(kw)
    return Beacon(**d)


def test_size_is_14_bytes_like_firmware():
    assert len(sample().pack()) == PACKET_SIZE == 14


def test_little_endian_layout_matches_header():
    raw = Beacon(3, EventType.VICTIM, 1.0, -1.0, next_hop=4, bearing_deg=90, age_s=1).pack()
    assert raw == bytes([1, 3, 2, 4]) + (100).to_bytes(2, "little") + (-100).to_bytes(2, "little", signed=True) \
        + (90).to_bytes(2, "little") + (1).to_bytes(4, "little")
    assert raw[0] == PKT_BEACON


def test_round_trip():
    r = Beacon.unpack(sample().pack())
    assert (r.beacon_id, r.event_type, r.next_hop, r.bearing_deg, r.age_s) == (7, EventType.GAS, 8, 270, 120)
    assert r.x == pytest.approx(12.34, abs=0.005) and r.y == pytest.approx(-5.67, abs=0.005)


def test_default_next_hop_is_none():
    assert Beacon.unpack(Beacon(1, EventType.NONE, 0, 0).pack()).next_hop == NONE_ID


def test_bad_packets_rejected():
    for bad in (b"\x00" * 13, b"\x02" + sample().pack()[1:], sample().pack()[:2] + b"\x09" + sample().pack()[3:]):
        with pytest.raises(PacketError):
            Beacon.unpack(bad)


def test_out_of_range_rejected():
    for kw in (dict(x=400.0), dict(beacon_id=0), dict(bearing_deg=360), dict(age_s=-1)):
        with pytest.raises(PacketError):
            sample(**kw).pack()


def test_aging_law_matches_firmware_constants():
    assert confidence(EventType.GAS, 0) == 1.0
    assert confidence(EventType.GAS, 600) == pytest.approx(math.exp(-1))
    assert confidence(EventType.VICTIM, 1800) == pytest.approx(math.exp(-1))
    assert confidence(EventType.NONE, 10**7) == 1.0          # waypoints do not decay


def test_stale_threshold_is_0_3():
    assert not sample(age_s=600).is_stale()                  # 0.37
    assert sample(age_s=900).is_stale()                      # 0.22
    assert not sample(event_type=EventType.VICTIM, age_s=900).is_stale()   # 0.61


def test_log_record_carries_parent():
    b, parent = unpack_log(pack_log(sample(), 5))
    assert parent == 5 and b.beacon_id == 7


def test_mission_round_trip_and_size():
    route = [(1, 0.0, 0.0), (3, 20.0, 0.0), (11, 40.0, -12.0)]
    raw = pack_mission(EventType.VICTIM, route)
    assert len(raw) == 3 + 5 * 3
    t, r = unpack_mission(raw)
    assert t == EventType.VICTIM and r == route
    with pytest.raises(PacketError):
        pack_mission(EventType.VICTIM, [(1, 0, 0)] * 51)     # would exceed 255 B
