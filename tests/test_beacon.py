import math
import pytest

from livingmap.beacon import (LOG_SIZE, MAX_ROUTE, NONE_ID, PACKET_SIZE, PKT_BEACON, PKT_LOG, PKT_MISSION,
                              Beacon, EventType, PacketError, confidence, pack_log, pack_mission,
                              unpack_log, unpack_mission)


def sample(**kw):
    d = dict(beacon_id=7, event_type=EventType.GAS, x=12.34, y=-5.67, next_hop=8,
             bearing_deg=270, age_s=120)
    d.update(kw)
    return Beacon(**d)


def test_matches_firmware_golden_frame():
    # Produced by compiling firmware/include/beacon_msg.h with g++ and building the frame like
    # firmware/src/beacon: BeaconMsg m = {1, 7, EVT_GAS, 8, 1234, -567, 270, 120}
    golden = bytes.fromhex("0101070208d204c9fd0e0178000000")
    assert sample().pack() == golden
    assert Beacon.unpack(golden) == Beacon(7, EventType.GAS, 12.34, -5.67, 8, 270, 120)


def test_sizes_match_firmware():
    assert len(sample().pack()) == PACKET_SIZE == 15
    assert len(pack_log(sample(), 5)) == LOG_SIZE == 16


def test_firmware_constants():
    assert (PKT_BEACON, PKT_LOG, PKT_MISSION) == (1, 2, 3)
    assert (EventType.NONE, EventType.VICTIM, EventType.GAS, EventType.BLOCKED) == (0, 1, 2, 3)


def test_little_endian_layout():
    raw = Beacon(3, EventType.VICTIM, 1.0, -1.0, next_hop=4, bearing_deg=90, age_s=1).pack()
    assert raw == bytes([1, 1, 3, 1, 4]) + (100).to_bytes(2, "little") \
        + (-100).to_bytes(2, "little", signed=True) + (90).to_bytes(2, "little") + (1).to_bytes(4, "little")


def test_round_trip():
    r = Beacon.unpack(sample().pack())
    assert (r.beacon_id, r.event_type, r.next_hop, r.bearing_deg, r.age_s) == (7, EventType.GAS, 8, 270, 120)
    assert r.x == pytest.approx(12.34, abs=0.005) and r.y == pytest.approx(-5.67, abs=0.005)


def test_default_next_hop_is_none():
    assert Beacon.unpack(Beacon(1, EventType.NONE, 0, 0).pack()).next_hop == NONE_ID


def test_bad_packets_rejected():
    good = sample().pack()
    bad_frames = (good[:-1],                       # wrong length
                  b"\x02" + good[1:],              # wrong packet type
                  good[:1] + b"\x09" + good[2:],   # unsupported version
                  good[:3] + b"\x09" + good[4:])   # unknown event type
    for bad in bad_frames:
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
    with pytest.raises(PacketError):
        unpack_log(sample().pack() + b"\x00")                # a BEACON frame is not a LOG frame


def test_mission_round_trip_and_size():
    route = [(1, 0.0, 0.0), (3, 20.0, 0.0), (11, 40.0, -12.0)]
    raw = pack_mission(EventType.VICTIM, route)
    assert raw[0] == PKT_MISSION and len(raw) == 3 + 5 * 3
    t, r = unpack_mission(raw)
    assert t == EventType.VICTIM and r == route


def test_mission_cap_is_48_entries():
    assert len(pack_mission(EventType.VICTIM, [(1, 0, 0)] * MAX_ROUTE)) == 243
    with pytest.raises(PacketError):
        pack_mission(EventType.VICTIM, [(1, 0, 0)] * (MAX_ROUTE + 1))
