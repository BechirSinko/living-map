"""Run the Living Map simulation.  py run_demo.py [--seed N] [--gif demo/demo.gif] [--no-show]"""
import argparse
from livingmap.sim import run_simulation, animate

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--gif", default=None, help="save animation to this .gif")
ap.add_argument("--no-show", action="store_true", help="do not open a window")
a = ap.parse_args()

sim = run_simulation(seed=a.seed)
print("\n=== SUMMARY ===")
print(f"Writer drift at return : {sim.writer_error:.2f} m")
print(f"Beacons dropped        : {len(sim.beacons)}")
print(f"Victim reached         : {sim.victim_reached}")
print(f"Executor gas exposure  : {sim.executor_exposure} steps (0 = avoided)")
animate(sim, every=6, save=a.gif, show=not a.no_show)
