# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Run a posted URScript program on a UR controller simulator (URSim) and compare the joints the controller reaches
with the joints lithocam planned.

URSim is Universal Robots' controller software; the e-Series image runs in Docker, for example

    docker run -d -p 29999:29999 -p 30001-30004:30001-30004 -p 6080:6080 -e ROBOT_MODEL=UR10 \
        universalrobots/ursim_e-series:<version>

(ROBOT_MODEL=UR10 is the UR10e in the e-Series image). The check:

1. powers the simulated robot on and releases the brakes (dashboard server, port 29999);
2. makes a simulation copy of the program: blocking popups become comments (nobody is there to press continue),
   and the TCP speeds of linear moves are multiplied by `speed_scale` up to `max_speed` (the joint solutions a
   controller chooses do not depend on the speed; blends of 0.5 mm change the path by less than the tolerance);
3. sends it to the secondary interface (port 30002) in chunks of a few hundred moves (the interface does not start
   a program of thousands of lines; each chunk repeats the TCP and payload lines), waits for each chunk's
   PROGRAM_XXX_STOPPED message on the primary interface (port 30001; the dashboard's "running" does not cover
   programs sent this way), and records q_actual from the real-time interface (port 30003, 500 Hz on e-Series);
4. for every planned waypoint (the joints of movej and movel lines, and the `# q [...]` note of movep and pose
   movel lines), finds the closest recorded sample at or after the previous match and reports the largest joint
   difference. A waypoint the controller passes in another configuration shows up as a large difference.

The robot program itself is unchanged; only the copy sent to the simulator is edited. Hardware stays out of scope.
"""
from __future__ import annotations

import re
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

_Q_OFFSET = 252          # q_actual in the real-time packet: after size, time and five 6-vectors of targets
_NUM = r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?"
_VEC = r"\[\s*(" + _NUM + r"(?:\s*,\s*" + _NUM + r"){5})\s*\]"


def dashboard(cmd: str, host: str = "127.0.0.1", port: int = 29999, timeout: float = 5.0) -> str:
    with socket.create_connection((host, port), timeout=timeout) as s:
        f = s.makefile("rw", newline="\n")
        f.readline()
        f.write(cmd + "\n")
        f.flush()
        return f.readline().strip()


def ready(host: str = "127.0.0.1") -> str:
    """Power on and release the brakes; returns the robot mode."""
    for c in ("close popup", "close safety popup", "power on"):
        dashboard(c, host)
    for _ in range(30):
        if "IDLE" in dashboard("robotmode", host) or "RUNNING" in dashboard("robotmode", host):
            break
        time.sleep(1)
    dashboard("brake release", host)
    for _ in range(30):
        m = dashboard("robotmode", host)
        if "RUNNING" in m:
            return m
        time.sleep(1)
    return dashboard("robotmode", host)


def waypoints(script: str) -> np.ndarray:
    """Planned joints in program order (rad): movej/movel joint lists and the `# q [...]` notes of pose moves."""
    out = []
    for line in script.splitlines():
        s = line.strip()
        if not s.startswith(("movej(", "movel(", "movep(")):
            continue
        m = re.search(r"#\s*q\s*" + _VEC, s)
        if m:
            out.append([float(v) for v in m.group(1).split(",")])
            continue
        m = re.match(r"move[jl]\(\s*" + _VEC, s)
        if m:
            out.append([float(v) for v in m.group(1).split(",")])
    return np.asarray(out, dtype=float).reshape(-1, 6)


def sim_copy(script: str, speed_scale: float = 1.0, max_speed: float = 0.25) -> str:
    """The program as sent to the simulator: popups commented out, linear TCP speeds scaled (movel, movep)."""
    out = []
    for line in script.splitlines():
        s = line.strip()
        if s.startswith("popup("):
            out.append(line.replace("popup(", "# [simulation copy] popup(", 1))
            continue
        if speed_scale != 1.0 and s.startswith(("movel(", "movep(")):
            line = re.sub(r"v=(" + _NUM + ")", lambda m_: "v={:.4f}".format(
                min(float(m_.group(1)) * speed_scale, max(max_speed, float(m_.group(1))))), line, count=1)
        out.append(line)
    return "\n".join(out) + "\n"


@dataclass
class Recording:
    t: List[float] = field(default_factory=list)
    q: List[List[float]] = field(default_factory=list)


def _record(host: str, rec: Recording, stop: threading.Event):
    with socket.create_connection((host, 30003), timeout=5.0) as s:
        buf = b""
        while not stop.is_set():
            buf += s.recv(65536)
            while len(buf) >= 4:
                n = struct.unpack_from(">i", buf, 0)[0]
                if len(buf) < n:
                    break
                pkt, buf = buf[:n], buf[n:]
                if n >= _Q_OFFSET + 48:
                    rec.t.append(struct.unpack_from(">d", pkt, 4)[0])
                    rec.q.append(list(struct.unpack_from(">6d", pkt, _Q_OFFSET)))


@dataclass
class RunReport:
    waypoints: int
    samples: int
    max_err_deg: float
    worst_index: int
    over_tol: int
    tol_deg: float
    seconds: float
    finished: bool
    errors_deg: np.ndarray

    def summary(self) -> str:
        return ("URSim: {} waypoints, {} samples in {:.0f} s, {}; largest joint difference {:.3f} deg at waypoint {}; "
                "{} waypoints over {:g} deg -> {}".format(
                    self.waypoints, self.samples, self.seconds, "program finished" if self.finished else "TIMED OUT",
                    self.max_err_deg, self.worst_index, self.over_tol, self.tol_deg,
                    "PASS" if self.finished and self.over_tol == 0 else "FAIL"))


def compare(planned: np.ndarray, recorded: np.ndarray, window: int = 20000, tol_deg: float = 0.5) -> np.ndarray:
    """Per planned waypoint, the max-joint difference (deg) to the recorded path, in order: the match is the FIRST
    sample within `tol_deg` at or after the previous match (within `window` samples), and the pointer moves there.
    A waypoint with no sample within tolerance gets its smallest difference and the pointer stays, so a near-miss
    never jumps ahead to a later pass with similar joints (comb slots repeat the same joint pattern)."""
    errs = np.empty(len(planned))
    tol = np.radians(tol_deg)
    j = 0
    for i, w in enumerate(planned):
        seg = recorded[j: j + window]
        if not len(seg):
            errs[i:] = np.inf
            break
        d = np.abs(seg - w).max(axis=1)
        hit = np.flatnonzero(d <= tol)
        if len(hit):
            k = int(hit[0])
            out = np.flatnonzero(d[k:] > tol)
            end = k + (int(out[0]) if len(out) else len(d) - k)
            k += int(np.argmin(d[k:end]))              # the closest sample of that pass within tolerance
            errs[i] = d[k]
            j += k
        else:
            errs[i] = d.min()
    return np.degrees(errs)


_SETUP = ("set_tcp(", "set_payload(", "set_tool_voltage(", "set_gravity(")


def chunks(script: str, moves_per_chunk: int = 150) -> List[str]:
    """The program (one `def NAME(): ... end NAME()`) split into programs of at most `moves_per_chunk` motion lines
    each, for the secondary interface, which does not start a program of thousands of lines. Every chunk repeats the
    setup lines seen so far (TCP, payload): a new program starts from the installation's TCP. A blend at the end of a
    chunk becomes a stop, which changes no joint target."""
    lines = script.splitlines()
    body = [ln for ln in lines[1:] if ln.strip() and not ln.startswith("end") and not re.match(r"^\w+\(\)\s*$", ln)]
    out, cur, setup, n = [], [], {}, 0
    for ln in body:
        s = ln.strip()
        for key in _SETUP:
            if s.startswith(key):
                setup[key] = ln
        cur.append(ln)
        if s.startswith(("movej(", "movel(", "movep(", "movec(")):
            n += 1
        if n >= moves_per_chunk:
            out.append(cur)
            cur, n = [], 0
    if cur:
        out.append(cur)
    progs = []
    for k, c in enumerate(out):
        pre = [v for key, v in setup.items() if k > 0 and v not in c]
        name = "lc_chunk_{}".format(k)
        progs.append("def {}():\n{}\nend\n{}()\n".format(name, "\n".join(pre + c), name))
    return progs


class _Primary:
    """Text of the robot messages on the primary interface (port 30001): PROGRAM_XXX_STARTED / STOPPED name the
    program, and a compile or runtime error shows up there too."""

    def __init__(self, host: str):
        self.buf = b""
        self.stop = threading.Event()
        self.s = socket.create_connection((host, 30001), timeout=5.0)
        self.s.settimeout(0.3)
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def _loop(self):
        while not self.stop.is_set():
            try:
                self.buf += self.s.recv(65536)
            except socket.timeout:
                pass
            except OSError:
                break

    def seen(self, text: bytes) -> bool:
        return text in self.buf

    def errors(self) -> List[str]:
        msgs = {m.decode(errors="ignore") for m in re.findall(rb"[ -~]{8,}", self.buf)}
        return sorted(m for m in msgs if any(k in m.lower() for k in ("error", "exception", "compile", "unknown",
                                                                      "not defined", "protective", "fault")))

    def close(self):
        self.stop.set()
        self.th.join(timeout=2)
        self.s.close()


def run(script: str, host: str = "127.0.0.1", speed_scale: float = 30.0, max_speed: float = 0.25,
        tol_deg: float = 0.5, timeout: float = 3600.0, start_timeout: float = 120.0,
        moves_per_chunk: int = 150, max_chunks: int = 0, save: str = "") -> RunReport:
    """Run the program on URSim in chunks and compare (see the module docstring)."""
    dashboard("stop", host)
    ready(host)
    planned = waypoints(script)
    progs = chunks(sim_copy(script, speed_scale, max_speed), moves_per_chunk)
    if max_chunks:
        progs = progs[:max_chunks]
    rec, stop = Recording(), threading.Event()
    th = threading.Thread(target=_record, args=(host, rec, stop), daemon=True)
    th.start()
    prim = _Primary(host)
    time.sleep(0.5)
    t0 = time.time()
    finished, i0 = True, None
    for k, prog in enumerate(progs):
        name = "lc_chunk_{}".format(k).encode()
        time.sleep(1.0)                 # a program sent right after the previous one stopped can be ignored
        if i0 is None:
            i0 = len(rec.q)             # the recording starts when the first chunk is sent
        with socket.create_connection((host, 30002), timeout=5.0) as s:
            s.sendall(prog.encode("utf-8"))
            time.sleep(2.0)             # closing at once can cut the program short while the controller reads it
        t1 = time.time()
        while not prim.seen(b"PROGRAM_XXX_STARTED" + name) and time.time() - t1 < start_timeout:
            time.sleep(0.2)
        if not prim.seen(b"PROGRAM_XXX_STARTED" + name):
            finished = False
            break
        while not prim.seen(b"PROGRAM_XXX_STOPPED" + name) and time.time() - t0 < timeout:
            time.sleep(0.2)
        if not prim.seen(b"PROGRAM_XXX_STOPPED" + name):
            finished = False
            break
    errs_txt = prim.errors()
    prim.close()
    time.sleep(0.3)
    stop.set()
    th.join(timeout=5)
    if errs_txt:
        finished = False
        print("URSim messages:", "; ".join(errs_txt[:5]))
    q = np.asarray(rec.q[i0 or 0:], dtype=float).reshape(-1, 6)
    if save:
        np.savez_compressed(save, q=q, t=np.asarray(rec.t[i0 or 0:]), planned=planned)
    errs = compare(planned, q, tol_deg=tol_deg) if len(q) and len(planned) else np.full(len(planned), np.inf)
    worst = int(np.argmax(errs)) if len(errs) else -1
    return RunReport(len(planned), len(q), float(errs.max()) if len(errs) else float("nan"), worst,
                     int((errs > tol_deg).sum()), tol_deg, time.time() - t0, finished, errs)


def main(argv=None):
    import argparse
    import json
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("script")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--speed-scale", type=float, default=30.0)
    ap.add_argument("--max-speed", type=float, default=0.25, help="m/s cap for scaled linear moves")
    ap.add_argument("--tol", type=float, default=0.5, help="deg")
    ap.add_argument("--timeout", type=float, default=3600.0)
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    with open(a.script, encoding="utf-8") as f:
        text = f.read()
    r = run(text, a.host, a.speed_scale, a.max_speed, a.tol, a.timeout)
    print(r.summary())
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({k: v for k, v in r.__dict__.items() if k != "errors_deg"} |
                      {"errors_deg": [round(float(e), 4) for e in r.errors_deg]}, f, indent=1)
    return 0 if r.finished and r.over_tol == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
