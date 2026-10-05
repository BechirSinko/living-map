"""End-to-end simulation: Writer -> beacons -> gateway (ONA) -> command post -> Executor.

Uses the repo packet code (livingmap/beacon.py) and frame code (livingmap/frames.py).
"""
import math
import random
from dataclasses import replace

from livingmap.beacon import (Beacon, EventType, NONE_ID, STALE_THRESHOLD,
                              pack_log, unpack_log, pack_mission, unpack_mission)
from livingmap.frames import local_to_gps as _l2g

# Beacon format = firmware/include/beacon_msg.h (14 B little-endian, age counter, no clock sync).
WAYPOINT, GAS, VICTIM = EventType.NONE, EventType.GAS, EventType.VICTIM
NAMES = {WAYPOINT: "waypoint", GAS: "GAS", VICTIM: "VICTIM"}
NONE = NONE_ID
RADIO_LOSS = 0.10     # frames lost to the SX1276 hardware CRC (radio drops them)
LAT0, LON0, HEADING = 36.8065, 10.1815, 40.0   # demo anchor: entrance GPS + bearing of +x


def local_to_gps(x, y):
    return _l2g(x, y, LAT0, LON0, HEADING)


# ---------------- tunnel world (metres, entrance = origin) ----------------
NODES = {"E": (0, 0), "J1": (20, 0), "J2": (40, 0), "END": (60, 0),
         "A": (20, 15), "P": (30, 10), "V": (40, -12)}
EDGES = [("E", "J1"), ("J1", "A"), ("J1", "J2"), ("J1", "P"), ("P", "J2"), ("J2", "V"), ("J2", "END")]
ADJ = {n: [] for n in NODES}
for a, b in EDGES:
    ADJ[a].append(b)
    ADJ[b].append(a)
GAS_C, GAS_R, GAS_SENSE = (30.0, 0.0), 3.0, 5.0     # gas plume sits on the direct J1-J2 corridor
VICTIM_POS, VICTIM_SENSE = (40.0, -12.5), 3.0
STEP, SPEED = 0.5, 0.5          # metres per step, m/s  -> 1 step = 1 s
BEACON_EVERY, RADIO = 10.0, 6.0  # drop a waypoint every N m, beacon radio range


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------- messaging: nothing talks to the command post except via the gateway ----------------
ALLOWED = {("gateway", "writer"), ("writer", "gateway"), ("gateway", "executor"),
           ("executor", "gateway"), ("gateway", "command_post"), ("command_post", "gateway")}


class Bus:
    def __init__(self, sim):
        self.sim, self.h, self.log = sim, {}, []

    def send(self, src, dst, kind, payload):
        if (src, dst) not in ALLOWED:
            raise PermissionError(f"direct link {src}->{dst} is forbidden (must go via gateway/ONA)")
        self.log.append((src, dst, kind))
        self.sim.say(f"{src:>12} -> {dst:<12} [{kind}]")
        self.h[dst](src, kind, payload)


class Sim:
    def __init__(self, verbose=True):
        self.t, self.verbose = 0.0, verbose
        self.beacons, self.frames, self.phase = [], [], "init"
        self.cp_state, self.exec_active = 0, False
        self.writer = self.executor = None
        self.cp_points, self.cp_route = [], []

    def say(self, msg):
        if self.verbose:
            print(f"[t={self.t:7.0f}s] {msg}")

    def snap(self):
        if self.writer is None:
            return
        e = tuple(self.executor.true) if self.exec_active else None
        self.frames.append((self.t, self.phase, tuple(self.writer.true), tuple(self.writer.est),
                            e, len(self.beacons), self.cp_state))


# ---------------- robots ----------------
class Robot:
    def __init__(self, sim, rng):
        self.sim, self.rng = sim, rng
        self.true, self.est, self.bias, self.heading = [0.0, 0.0], [0.0, 0.0], 0.0, 0

    def move(self, dx, dy):
        """True displacement; the robot's own position estimate drifts (dead reckoning)."""
        self.true[0] += dx
        self.true[1] += dy
        self.bias += self.rng.gauss(0, 0.002)
        k = 1 + self.rng.gauss(0, 0.01)
        c, s = math.cos(self.bias + 0.01), math.sin(self.bias + 0.01)
        ex, ey = k * (c * dx - s * dy), k * (s * dx + c * dy)
        self.est[0] += ex
        self.est[1] += ey
        self.heading = round(math.degrees(math.atan2(ey, ex))) % 360
        self.sim.t += STEP / SPEED
        self.sim.snap()

    def walk(self, target, on_step=None):
        while True:
            dx, dy = target[0] - self.true[0], target[1] - self.true[1]
            d = math.hypot(dx, dy)
            if d < 1e-6:
                return False
            k = min(STEP, d) / d
            self.move(dx * k, dy * k)
            if on_step and on_step(self):
                return True
            if d <= STEP:
                self.true = [float(target[0]), float(target[1])]
                return False


class Writer(Robot):
    def __init__(self, sim, rng, bus):
        super().__init__(sim, rng)
        self.bus = bus
        self.next_id, self.prev, self.since = 1, NONE, 0.0
        self.node_beacon, self.visited, self.log = {}, set(), []

    def on_msg(self, src, kind, p):
        pass

    def drop(self, etype):
        bc = Beacon(self.next_id, etype, self.est[0], self.est[1],
                    next_hop=self.next_id + 1, bearing_deg=self.heading)
        self.sim.beacons.append({"id": bc.beacon_id, "true": tuple(self.true), "etype": etype,
                                 "beacon": bc, "t_drop": self.sim.t})
        self.log.append((bc, self.prev, self.sim.t))
        self.sim.say(f"Writer drops beacon #{bc.beacon_id:<2} {NAMES[etype]:<8} at est=({self.est[0]:5.1f},{self.est[1]:5.1f}) "
                     f"true=({self.true[0]:5.1f},{self.true[1]:5.1f}) heading={bc.bearing_deg}")
        self.prev, self.since, self.next_id = bc.beacon_id, 0.0, bc.beacon_id + 1
        return bc.beacon_id

    def run(self):
        self.sim.phase = "Writer exploring"
        self.drop(WAYPOINT)
        self.node_beacon["E"] = self.prev
        self.dfs("E")
        err = dist(self.est, self.true)
        self.sim.say(f"Writer back at entrance, dead-reckoning error = {err:.2f} m")
        self.sim.phase = "Writer uploading log"
        recs = [pack_log(replace(bc, age_s=int(self.sim.t - t)), parent).hex() for bc, parent, t in self.log]
        self.bus.send("writer", "gateway", "log_upload", recs)
        return err

    def dfs(self, node):
        for nb in ADJ[node]:
            e = frozenset((node, nb))
            if e in self.visited:
                continue
            self.visited.add(e)
            self.prev, self.since = self.node_beacon[node], 0.0
            if self.walk_edge(node, nb):
                continue                                    # gas: turned back
            if nb in self.node_beacon:                      # loop closure
                self.walk(NODES[node])
                continue
            et = VICTIM if dist(NODES[nb], VICTIM_POS) <= VICTIM_SENSE else WAYPOINT
            self.node_beacon[nb] = self.drop(et)
            self.dfs(nb)
            self.walk(NODES[node])                          # backtrack, no beacons

    def walk_edge(self, a, b):
        tgt = NODES[b]

        def on_step(r):
            if dist(r.true, GAS_C) <= GAS_SENSE:
                return True
            self.since += STEP
            if self.since >= BEACON_EVERY and dist(r.true, tgt) >= 4:
                self.drop(WAYPOINT)
            return False

        if self.walk(tgt, on_step):
            self.drop(GAS)                # value = gas level
            self.walk(NODES[a])                             # retreat, never enter the plume
            return True
        return False


class Gateway:
    """Outside Network Area: LoRa side <-> Wi-Fi/MQTT side. Does frame translation."""

    def __init__(self, bus, sim):
        self.bus, self.sim = bus, sim

    def on_msg(self, src, kind, p):
        if kind == "log_upload":
            pts = []
            for h in p:
                try:
                    k, parent = unpack_log(bytes.fromhex(h))
                except ValueError:
                    self.sim.say("gateway: dropped malformed record")
                    continue
                lat, lon = local_to_gps(k.x, k.y)
                pts.append({"id": k.beacon_id, "etype": k.event_type, "prev": parent, "x": k.x, "y": k.y,
                            "lat": lat, "lon": lon, "conf": k.aged_confidence(), "age_s": k.age_s})
            self.bus.send("gateway", "command_post", "map_update", pts)
        elif kind == "mission":
            self.bus.send("gateway", "executor", "briefing", p)   # p = MISSION bytes, forwarded as-is


class CommandPost:
    def __init__(self, bus, sim):
        self.bus, self.sim = bus, sim

    def on_msg(self, src, kind, p):
        if kind != "map_update":
            return
        self.sim.cp_points, self.sim.cp_state = p, 1
        by = {b["id"]: b for b in p}
        victims = [b for b in p if b["etype"] == VICTIM]
        if not victims:
            return
        path, cur = [], victims[0]["id"]
        while cur in by:                                    # follow parent edges back to the entrance
            path.append(by[cur])
            cur = by[cur]["prev"]
        path.reverse()
        self.sim.cp_route, self.sim.cp_state = path, 2
        gas = [b["id"] for b in p if b["etype"] == GAS]
        self.sim.say(f"command post: route {[b['id'] for b in path]} to victim, hazard beacons {gas} excluded")
        mission = pack_mission(VICTIM, [(b["id"], b["x"], b["y"]) for b in path])
        self.sim.say(f"command post: mission packet = {len(mission)} bytes")
        self.bus.send("command_post", "gateway", "mission", mission)


class Executor(Robot):
    def __init__(self, sim, rng, bus):
        super().__init__(sim, rng)
        self.bus, self.mission, self.heard, self.crc_fail = bus, None, {}, 0
        self.exposure, self.idx, self.corrections = 0, 1, []

    def on_msg(self, src, kind, p):
        if kind == "briefing":
            target, route = unpack_mission(p)
            self.mission = {"target": target, "route": [{"id": i, "x": x, "y": y} for i, x, y in route]}
            self.sim.say(f"Executor briefed at entrance: {len(route)} beacons ({len(p)} B mission packet)")

    def listen(self):
        for b in self.sim.beacons:
            if b["id"] in self.heard or dist(self.true, b["true"]) > RADIO:
                continue
            if self.rng.random() < RADIO_LOSS:                  # hardware CRC failed: radio drops the frame
                self.crc_fail += 1
                continue
            age = int(self.sim.t - b["t_drop"])                 # the beacon's own age counter
            p = Beacon.unpack(replace(b["beacon"], age_s=age).pack())
            c = p.aged_confidence()
            self.heard[p.beacon_id] = c
            tag = "STALE-verify" if c < STALE_THRESHOLD else "fresh"
            self.sim.say(f"Executor hears #{p.beacon_id} {NAMES[p.event_type]} age={p.age_s}s conf={c:.2f} ({tag})"
                         + ("  HAZARD, not on route" if p.event_type == GAS else ""))

    def run(self):
        self.sim.phase = "Executor navigating"
        self.sim.exec_active = True
        route, self.est, self.node, self.came = self.mission["route"], [0.0, 0.0], "E", ""
        beacon = {b["id"]: b for b in self.sim.beacons}

        def on_step(r):
            self.listen()
            if dist(r.true, GAS_C) < GAS_R:
                self.exposure += 1
            wp = route[self.idx]
            if dist(r.true, beacon[wp["id"]]["true"]) <= 1.0:   # strong RSSI = passing the beacon
                self.corrections.append(dist(self.est, (wp["x"], wp["y"])))
                self.est = [wp["x"], wp["y"]]                    # re-anchor to beacon coordinates
                self.sim.say(f"Executor passes #{wp['id']}, re-anchors (drift fixed: {self.corrections[-1]:.2f} m)")
                self.idx += 1
                return self.idx >= len(route)
            return False

        while self.idx < len(route):
            t = route[self.idx]
            vx, vy = t["x"] - self.est[0], t["y"] - self.est[1]
            cand = [n for n in ADJ[self.node] if n != self.came] or ADJ[self.node]

            def score(n):
                ex, ey = NODES[n][0] - NODES[self.node][0], NODES[n][1] - NODES[self.node][1]
                return (ex * vx + ey * vy) / (math.hypot(ex, ey) * math.hypot(vx, vy) + 1e-9)

            nb = max(cand, key=score)
            if self.walk(NODES[nb], on_step):
                break
            self.came, self.node = self.node, nb
        self.sim.phase = "Mission complete: victim reached"
        self.sim.say("Executor reached the victim beacon. MISSION COMPLETE.")


def run_simulation(seed=7, verbose=True, exec_delay_s=900):
    rng = random.Random(seed)
    sim = Sim(verbose)
    bus = Bus(sim)
    gw, cp = Gateway(bus, sim), CommandPost(bus, sim)
    wr, ex = Writer(sim, rng, bus), Executor(sim, rng, bus)
    sim.writer, sim.executor = wr, ex
    bus.h = {"gateway": gw.on_msg, "command_post": cp.on_msg, "writer": wr.on_msg, "executor": ex.on_msg}
    sim.writer_error = wr.run()
    sim.t += exec_delay_s                                   # Executor is deployed later (beacon age counters keep running)
    sim.say(f"--- Executor deployed {exec_delay_s // 60} min after the Writer finished ---")
    ex.run()
    sim.executor_exposure, sim.bus = ex.exposure, bus
    sim.victim_reached = ex.idx >= len(ex.mission["route"])
    return sim


# ---------------- visualisation ----------------
def animate(sim, every=3, save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import numpy as np

    fig, (ax, cx) = plt.subplots(1, 2, figsize=(13, 6))
    for a, b in EDGES:
        ax.plot(*zip(NODES[a], NODES[b]), color="#888", lw=8, solid_capstyle="round", alpha=.4)
    ax.add_patch(plt.Circle(GAS_C, GAS_R, color="orange", alpha=.35))
    ax.plot(*VICTIM_POS, "r*", ms=16)
    ax.set_title("Mine (ground truth)"); ax.set_aspect("equal"); ax.set_xlim(-5, 65); ax.set_ylim(-18, 20)
    col = {WAYPOINT: "#555", GAS: "orange", VICTIM: "red"}
    bsc = ax.scatter([], [], s=45, zorder=3)
    wdot, = ax.plot([], [], "bo", ms=10, label="Writer"); edot, = ax.plot([], [], "gs", ms=10, label="Executor")
    ax.legend(loc="upper right")
    txt = ax.text(-4, 17.5, "", fontsize=10, va="top")
    cx.set_title("Command post: live map (GPS)"); cx.tick_params(labelsize=7)
    allp = [local_to_gps(*NODES[n]) for n in NODES]
    cx.set_xlim(min(p[1] for p in allp) - 3e-4, max(p[1] for p in allp) + 3e-4)
    cx.set_ylim(min(p[0] for p in allp) - 2e-4, max(p[0] for p in allp) + 2e-4)
    csc = cx.scatter([], [], s=45); rline, = cx.plot([], [], "g-", lw=2, label="mission route")
    cx.text(.5, .5, "", transform=cx.transAxes)
    fr = sim.frames[::every] + [sim.frames[-1]]

    def upd(i):
        t, ph, wt, we, et, nb, cps = fr[i]
        bs = sim.beacons[:nb]
        bsc.set_offsets(np.array([b["true"] for b in bs]).reshape(-1, 2))
        bsc.set_color([col[b["etype"]] for b in bs])
        wdot.set_data([wt[0]], [wt[1]])
        edot.set_data([et[0]] if et else [], [et[1]] if et else [])
        txt.set_text(f"t = {t:5.0f} s\n{ph}")
        pts = sim.cp_points if cps >= 1 else []
        csc.set_offsets(np.array([[p["lon"], p["lat"]] for p in pts]).reshape(-1, 2))
        csc.set_color([col[p["etype"]] for p in pts])
        r = sim.cp_route if cps >= 2 else []
        rline.set_data([p["lon"] for p in r], [p["lat"] for p in r])
        return bsc, wdot, edot, txt, csc, rline

    ani = FuncAnimation(fig, upd, frames=len(fr), interval=40, blit=False)
    if save:
        ani.save(save, writer="pillow", fps=15, dpi=60)
    if show:
        plt.show()
    plt.close(fig)

