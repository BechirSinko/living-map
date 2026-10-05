"""Save a static snapshot of the simulated mine for the report."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from livingmap.sim import run_simulation, NODES, EDGES, GAS_C, GAS_R, VICTIM_POS, WAYPOINT, GAS, VICTIM

sim = run_simulation(seed=7, verbose=False)

fig, ax = plt.subplots(figsize=(8, 5))
for a, b in EDGES:
    ax.plot(*zip(NODES[a], NODES[b]), color="#888", lw=8, solid_capstyle="round", alpha=.4)
ax.add_patch(plt.Circle(GAS_C, GAS_R, color="orange", alpha=.35))
ax.plot(*VICTIM_POS, "r*", ms=16, label="victim")

col = {WAYPOINT: "#555", GAS: "orange", VICTIM: "red"}
for b in sim.beacons:
    ax.plot(*b["true"], "o", color=col[b["etype"]], ms=8, zorder=3)

route = [(p["x"], p["y"]) for p in sim.cp_route]
if route:
    ax.plot([p[0] for p in route], [p[1] for p in route], "g-", lw=2, label="Executor route")

ax.set_title("Simulated mine (seed 7)")
ax.set_aspect("equal")
ax.set_xlim(-5, 65); ax.set_ylim(-18, 20)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.legend(loc="upper right")
fig.tight_layout()
fig.savefig("docs/diagrams/simulation_result.png", dpi=150)
print("wrote docs/diagrams/simulation_result.png")