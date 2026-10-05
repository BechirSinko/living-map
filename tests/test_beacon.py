import pytest

from livingmap.beacon import (NONE_ID, PACKET_SIZE, Beacon, EventType, Flag,
                              PacketError, crc16_ccitt)


def sample(**kw):
    d = dict(beacon_id=7, event_type=EventType.GAS, x=12.34, y=-5.67, prev_id=6,
             next_id=8, timestamp=1_760_000_000, confidence=0.9, value=180,
             flags=Flag.JUNCTION | Flag.HAZARD_AHEAD)
    d.update(kw)
    return Beacon(**d)


def test_crc_check_vector():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_size_is_16_bytes():
    assert len(sample().pack()) == PACKET_SIZE == 16


def test_round_trip():
    b = sample()
    r = Beacon.unpack(b.pack())
    assert r.beacon_id == 7 and r.event_type == EventType.GAS
    assert r.x == pytest.approx(12.34, abs=0.005) and r.y == pytest.approx(-5.67, abs=0.005)
    assert (r.prev_id, r.next_id, r.timestamp, r.value, r.flags) == (6, 8, 1_760_000_000, 180, 0x5)
    assert r.confidence == pytest.approx(0.9, abs=1 / 255)


def test_defaults_use_none_id():
    r = Beacon.unpack(Beacon(1, EventType.WAYPOINT, 0, 0).pack())
    assert r.prev_id == NONE_ID and r.next_id == NONE_ID


@pytest.mark.parametrize("i", range(14))
def test_single_bit_corruption_detected(i):
    raw = bytearray(sample().pack())
    raw[i] ^= 0x01
    with pytest.raises(PacketError):
        Beacon.unpack(bytes(raw))


def test_wrong_length_rejected():
    with pytest.raises(PacketError):
        Beacon.unpack(b"\x00" * 15)


def test_out_of_range_rejected():
    with pytest.raises(PacketError):
        sample(x=400.0).pack()
    with pytest.raises(PacketError):
        sample(beacon_id=0).pack()
    with pytest.raises(PacketError):
        sample(value=300).pack()


def test_aging_halves_at_half_life():
    b = sample(confidence=1.0, timestamp=1000)
    assert b.aged_confidence(now=1000) == pytest.approx(1.0)
    assert b.aged_confidence(now=1600) == pytest.approx(0.5)  # gas half-life 600 s
    assert not b.is_stale(now=1000)
    assert b.is_stale(now=1000 + 1300)  # > 2 half-lives -> below 0.25


def test_event_types_age_differently():
    gas = sample(event_type=EventType.GAS, timestamp=0)
    victim = sample(event_type=EventType.VICTIM, timestamp=0)
    assert gas.aged_confidence(now=1800) < victim.aged_confidence(now=1800)
