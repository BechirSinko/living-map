import pytest
from livingmap.sim import run_simulation, GAS, VICTIM


@pytest.mark.parametrize("seed", [1, 7, 42])
def test_mission_succeeds_and_avoids_gas(seed):
    sim = run_simulation(seed=seed, verbose=False)
    assert sim.victim_reached
    assert sim.executor_exposure == 0
    assert {b["etype"] for b in sim.beacons} >= {GAS, VICTIM}


def test_no_direct_link_to_command_post():
    sim = run_simulation(seed=3, verbose=False)
    assert all("gateway" in (s, d) for s, d, _ in sim.bus.log)


def test_gas_beacon_is_stale_and_victim_fresh_after_delay():
    from livingmap.beacon import confidence, STALE_THRESHOLD, EventType
    sim = run_simulation(seed=7, verbose=False)
    gas = next(p for p in sim.cp_points if p["etype"] == GAS)
    victim = next(p for p in sim.cp_points if p["etype"] == VICTIM)
    # age at the gateway is small; after the 900 s delay gas is already stale, victim is not
    assert confidence(EventType.GAS, gas["age_s"] + 900) < STALE_THRESHOLD
    assert confidence(EventType.VICTIM, victim["age_s"] + 900) > STALE_THRESHOLD


def test_route_excludes_gas_beacon():
    sim = run_simulation(seed=7, verbose=False)
    assert all(p["etype"] != GAS for p in sim.cp_route)
