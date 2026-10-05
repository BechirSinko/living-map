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
