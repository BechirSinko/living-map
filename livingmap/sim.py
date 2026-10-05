"""End-to-end Phase 1 simulation: Writer -> beacons -> gateway (ONA) -> command post -> Executor.

Self-contained on purpose (packet + GPS maths are inlined, mirroring beacon.py / frames.py)
so the demo runs on its own. Later you can swap pack/unpack/local_to_gps for the repo versions.
"""
import math
import random
import struct
from dataclasses import dataclass

# ---------------- beacon packet (16 bytes, see docs/beacon_packet.md) ----------------
WAYPOINT, GAS, VICTIM, BLOCKED = 0, 1, 2, 3
F_JUNCTION, F_DEAD, F_HAZ, F_ENTR = 0x1, 0x2, 0x4, 0x8
NONE = 0xFF
HALF_LIFE = {WAYPOINT: 86400, GAS: 600, BLOCKED: 1800, VICTIM: 3600}
STALE = 0.25
NAMES = {WAYPOINT: "waypoint", GAS: "GAS", VICTIM: "VICTIM", BLOCKED: "blocked"}


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Pkt:
    id: int
    etype: int
    flags: int
    prev: int
    nxt: int
    x_cm: int
    y_cm: int
    ts: int
    conf: int
    value: int


def pack(p):
    body = struct.pack(">BBBBhhIBB", p.id, (p.etype << 4) | p.flags, p.prev, p.nxt,
                       p.x_cm, p.y_cm, p.ts, p.conf, p.value)
    return body + struct.pack(">H", crc16(body))


def unpack(raw):
    if len(raw) != 16 or crc16(raw[:14]) != struct.unpack(">H", raw[14:])[0]:
        raise ValueError("CRC/length error")
    i, tf, pr, nx, x, y, ts, c, v = struct.unpack(">BBBBhhIBB", raw[:14])
    return Pkt(i, tf >> 4, tf & 0xF, pr, nx, x, y, ts, c, v)


def aged_conf(p, now):
    return (p.conf / 255.0) * 0.5 ** (max(0, now - p.ts) / HALF_LIFE[p.etype])


# ---------------- frame translation (see docs/frame_translation.md) ----------------
LAT0, LON0, HEADING = 36.8065, 10.1815, 40.0   # demo anchor: entrance GPS + bearing of +x


def local_to_gps(x, y, lat0=LAT0, lon0=LON0, heading=HEADING):
    psi = math.radians(heading)
    east = x * math.sin(psi) - y * math.cos(psi)
    north = x * math.cos(psi) + y * math.sin(psi)
    ph = math.radians(lat0)
    m_lat = 111132.92 - 559.82 * math.cos(2 * ph) + 1.175 * math.cos(4 * ph)
    m_lon = 111412.84 * math.cos(ph) - 93.5 * math.cos(3 * ph)
    return lat0 + north / m_lat, lon0 + east / m_lon


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
        self.t, self.t0, self.verbose = 0.0, 1_790_000_000, verbose
        self.beacons, self.frames, self.phase = [], [], "init"
        self.cp_state, self.exec_active = 0, False
        self.writer = self.executor = None
        self.cp_points, self.cp_route = [], []

    def now(self):
        return self.t0 + int(self.t)

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
        self.true, self.est, self.bias = [0.0, 0.0], [0.0, 0.0], 0.0

    def move(self, dx, dy):
        """True displacement; the robot's own position estimate drifts (dead reckoning)."""
        self.true[0] += dx
        self.true[1] += dy
        self.bias += self.rng.gauss(0, 0.002)
        k = 1 + self.rng.gauss(0, 0.01)
        c, s = math.cos(self.bias + 0.01), math.sin(self.bias + 0.01)
        self.est[0] += k * (c * dx - s * dy)
        self.est[1] += k * (s * dx + c * dy)
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
        self.bus, self.clock_off = bus, 0
        self.next_id, self.prev, self.since = 1, NONE, 0.0
        self.node_beacon, self.visited, self.packets = {}, set(), []

    def on_msg(self, src, kind, p):
        if kind == "time_sync":
            self.clock_off = p["unix"] - self.sim.t0     # clock synced at the entrance

    def drop(self, etype, flags=0, value=0):
        p = Pkt(self.next_id, etype, flags, self.prev, NONE, round(self.est[0] * 100),
                round(self.est[1] * 100), self.sim.t0 + self.clock_off + int(self.sim.t), 255, value)
        raw = pack(p)
        self.sim.beacons.append({"id": p.id, "true": tuple(self.true), "etype": etype, "pkt": raw})
        self.packets.append(raw)
        self.sim.say(f"Writer drops beacon #{p.id:<2} {NAMES[etype]:<8} at est=({self.est[0]:5.1f},{self.est[1]:5.1f}) "
                     f"true=({self.true[0]:5.1f},{self.true[1]:5.1f})")
        self.prev, self.since, self.next_id = p.id, 0.0, p.id + 1
        return p.id

    def run(self):
        self.sim.phase = "Writer exploring"
        self.drop(WAYPOINT, F_ENTR)
        self.node_beacon["E"] = self.prev
        self.dfs("E")
        err = dist(self.est, self.true)
        self.sim.say(f"Writer back at entrance, dead-reckoning error = {err:.2f} m")
        self.sim.phase = "Writer uploading log"
        self.bus.send("writer", "gateway", "log_upload", [p.hex() for p in self.packets])
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
            deg, flags, et, val = len(ADJ[nb]), 0, WAYPOINT, 0
            if deg >= 3:
                flags |= F_JUNCTION
            if deg == 1:
                flags |= F_DEAD
            if dist(NODES[nb], VICTIM_POS) <= VICTIM_SENSE:
                et, val = VICTIM, 1
            self.node_beacon[nb] = self.drop(et, flags, val)
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
            self.drop(GAS, F_HAZ, value=180)                # value = gas level
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
                    k = unpack(bytes.fromhex(h))
                except ValueError:
                    self.sim.say("gateway: dropped packet (bad CRC)")
                    continue
                x, y = k.x_cm / 100, k.y_cm / 100
                lat, lon = local_to_gps(x, y)
                pts.append({"id": k.id, "etype": k.etype, "flags": k.flags, "prev": k.prev, "x": x, "y": y,
                            "lat": lat, "lon": lon, "conf": aged_conf(k, self.sim.now()), "value": k.value})
            self.bus.send("gateway", "command_post", "map_update", pts)
        elif kind == "mission":
            self.bus.send("gateway", "executor", "briefing", p)


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
        while cur in by:                                    # follow prev pointers back to the entrance
            path.append(by[cur])
            cur = by[cur]["prev"]
        path.reverse()
        self.sim.cp_route, self.sim.cp_state = path, 2
        gas = [b["id"] for b in p if b["etype"] == GAS]
        self.sim.say(f"command post: route {[b['id'] for b in path]} to victim, hazard beacons {gas} excluded")
        self.bus.send("command_post", "gateway", "mission", {
            "target": victims[0]["id"], "avoid": gas,
            "route": [{"id": b["id"], "x": b["x"], "y": b["y"]} for b in path]})


class Executor(Robot):
    def __init__(self, sim, rng, bus):
        super().__init__(sim, rng)
        self.bus, self.mission, self.heard, self.crc_fail = bus, None, {}, 0
        self.exposure, self.idx, self.corrections = 0, 1, []

    def on_msg(self, src, kind, p):
        if kind == "briefing":
            self.mission = p
            self.sim.say(f"Executor briefed at entrance: {len(p['route'])} beacons, avoid {p['avoid']}")

    def listen(self):
        for b in self.sim.beacons:
            if b["id"] in self.heard or dist(self.true, b["true"]) > RADIO:
                continue
            raw = bytearray(b["pkt"])
            if self.rng.random() < 0.1:
                raw[4] ^= 0x10                              # simulated RF bit error
            try:
                p = unpack(bytes(raw))
            except ValueError:
                self.crc_fail += 1
                continue
            c = aged_conf(p, self.sim.now())
            self.heard[b["id"]] = c
            tag = "STALE-verify" if c < STALE else "fresh"
            self.sim.say(f"Executor hears #{p.id} {NAMES[p.etype]} conf={c:.2f} ({tag})"
                         + ("  HAZARD, not on route" if p.flags & F_HAZ else ""))

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
    bus.send("gateway", "writer", "time_sync", {"unix": sim.t0})
    sim.writer_error = wr.run()
    sim.t += exec_delay_s                                   # Executor is deployed later
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
