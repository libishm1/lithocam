# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Run a URScript program in URSim (UR's controller software in Docker) and report what the controller did.

    python tools/ursim_run.py program.script [--host 127.0.0.1] [--start-docker] [--model UR10] [--timeout 300]

With --start-docker it runs (needs Docker; inside WSL use `wsl docker ...` or run this script inside WSL):
    docker run --rm -d --name lithocam_ursim -p 29999:29999 -p 30001-30004:30001-30004 -p 6080:6080 \
        -e ROBOT_MODEL=<model> universalrobots/ursim_e-series:5.26
(ROBOT_MODEL=UR10 gives the e-series UR10e; the web VNC is at http://localhost:6080/vnc.html.)

Steps: wait for the dashboard server (port 29999), power on, brake release, send the script to the primary
interface (port 30002), then poll `programState`, `safetystatus` and `robotmode` until the program stops.
Exit 0 when it ran to the end with safety status NORMAL, else 1. Simulation only: never point this at a real robot
without a hardware gate.
"""
from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time


def dash(host: str, cmd: str, timeout: float = 5.0) -> str:
    with socket.create_connection((host, 29999), timeout=timeout) as s:
        s.recv(4096)                                  # greeting
        s.sendall((cmd + "\n").encode())
        return s.recv(4096).decode(errors="replace").strip()


def wait_dashboard(host: str, timeout: float) -> bool:
    t = time.time()
    while time.time() - t < timeout:
        try:
            dash(host, "robotmode")
            return True
        except OSError:
            time.sleep(2)
    return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("script")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--start-docker", action="store_true")
    ap.add_argument("--docker", default="docker", help="docker command, e.g. 'wsl docker'")
    ap.add_argument("--model", default="UR10")
    ap.add_argument("--timeout", type=float, default=300)
    a = ap.parse_args(argv)
    if a.start_docker:
        cmd = a.docker.split() + ["run", "--rm", "-d", "--name", "lithocam_ursim", "-p", "29999:29999",
                                  "-p", "30001-30004:30001-30004", "-p", "6080:6080", "-e", "ROBOT_MODEL=" + a.model,
                                  "universalrobots/ursim_e-series:5.26"]
        print(" ".join(cmd))
        subprocess.run(cmd, check=True)
    if not wait_dashboard(a.host, 180):
        print("dashboard server not reachable on port 29999")
        return 1
    print("robotmode:", dash(a.host, "robotmode"))
    print(dash(a.host, "power on"))
    for _ in range(60):
        if "IDLE" in dash(a.host, "robotmode"):
            break
        time.sleep(1)
    print(dash(a.host, "brake release"))
    for _ in range(60):
        if "RUNNING" in dash(a.host, "robotmode"):
            break
        time.sleep(1)
    print("robotmode:", dash(a.host, "robotmode"), "| safety:", dash(a.host, "safetystatus"))
    prog = open(a.script, encoding="utf-8").read()
    with socket.create_connection((a.host, 30002), timeout=10) as s:
        s.sendall(prog.encode())
    t0 = time.time()
    started = False
    last = ""
    while time.time() - t0 < a.timeout:
        st = dash(a.host, "programState")
        safety = dash(a.host, "safetystatus")
        if st != last:
            print("{:7.1f} s  {} | {}".format(time.time() - t0, st, safety))
            last = st
        if "PLAYING" in st:
            started = True
        if "NORMAL" not in safety:
            print("SAFETY:", safety)
            return 1
        if started and "STOPPED" in st:
            print("program finished in {:.1f} s, safety {}".format(time.time() - t0, safety))
            return 0
        time.sleep(0.5)
    print("timeout; started" if started else "timeout; the program never started (check remote control mode)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
