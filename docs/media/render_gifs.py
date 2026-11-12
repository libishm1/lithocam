# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Render the docs GIFs from the data written by make_gif_data.py and make_robot_data.py (matplotlib + Pillow).

    python docs/media/render_gifs.py <data dir> <out dir>
"""
import io
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection  # noqa: E402
from PIL import Image  # noqa: E402

DATA, OUT = Path(sys.argv[1]), Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
COL = {"wire": "#3b82c4", "flank": "#e8913a", "point": "#9aa3ad"}


def grab(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=80)
    buf.seek(0)
    im = Image.open(buf).convert("RGB")
    return im.convert("P", palette=Image.ADAPTIVE, colors=128)


def save(frames, name, ms):
    path = OUT / name
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=ms, loop=0, optimize=True, disposal=2)
    print(name, len(frames), "frames", path.stat().st_size // 1024, "KB")


def equal_box(ax, pts, pad=0.02, zoom=1.25, min_frac=0.12):
    """True proportions, the box fitted to the data. Very thin directions are padded to min_frac of the largest
    extent so a slab does not render as a line."""
    pts = np.asarray(pts)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    ext = np.maximum(hi - lo, min_frac * (hi - lo).max()) * (1 + pad)
    c = (lo + hi) / 2
    ax.set_xlim(c[0] - ext[0] / 2, c[0] + ext[0] / 2)
    ax.set_ylim(c[1] - ext[1] / 2, c[1] + ext[1] / 2)
    ax.set_zlim(c[2] - ext[2] / 2, c[2] + ext[2] / 2)
    ax.set_box_aspect(tuple(ext), zoom=zoom)


def clean(ax):
    ax.set_axis_off()


# ---------------------------------------------------------------- 1) ruled labels
parts = [("twisted_column", "Twisted column: 4 ruled sides, all wire"),
         ("label_demo", "Block: through hole + chamfer wire, blind hole flank, slot point")]
data = {n: json.loads((DATA / "ruled_{}.json".format(n)).read_text()) for n, _ in parts}
frames = []
CLOSE = np.array([[60, -10, -5], [380, 140, 35]])     # cladding close-up: one anchor hole and one kerf slot
for k in range(30):
    fig = plt.figure(figsize=(9, 4.6))
    for i, (n, title) in enumerate(parts):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        allp = []
        for f in data[n]:
            tris = np.asarray(f["tris"])
            if len(tris):
                ax.add_collection3d(Poly3DCollection(tris, facecolor=COL[f["label"]], edgecolor="#22303c",
                                                     linewidths=0.05, alpha=0.95))
                allp.append(tris.reshape(-1, 3))
            if f["rulings"]:
                ax.add_collection3d(Line3DCollection(np.asarray(f["rulings"]), colors="#1d2733", linewidths=0.6))
        equal_box(ax, np.vstack(allp), zoom=1.0)
        ax.view_init(elev=32 if n == "label_demo" else 24, azim=-60 + 12 * k)
        clean(ax)
        ax.set_title(title, fontsize=10)
    fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.08, wspace=0)
    fig.text(0.5, 0.03, "lithocam.ruled: blue = wire saw, orange = flank milling, grey = point milling; "
             "dark lines = rulings", ha="center", fontsize=8.5)
    frames.append(grab(fig))
    plt.close(fig)
save(frames, "ruled_labels.gif", 110)

# ---------------------------------------------------------------- 2) milling simulation
m = np.load(DATA / "mill_tread.npz")
snaps, target, ext = m["snaps"], m["target"], m["extent"]
frames = []
for k, s in enumerate(list(snaps) + [snaps[-1]] * 6):
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    im = ax.imshow(np.clip(s - target, 0, 1.2), origin="lower", extent=ext, cmap="magma_r", vmin=0, vmax=1.2,
                   interpolation="nearest")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("Z-map simulation, ball R3, stepover {:.2f} mm (0.2 mm scallop): pass {}/{}".format(
        float(m["stepover"]), min(k + 1, len(snaps)), len(snaps)), fontsize=9)
    cb = fig.colorbar(im, ax=ax, fraction=0.035)
    cb.set_label("material above the part (mm)", fontsize=8)
    fig.tight_layout()
    frames.append(grab(fig))
    plt.close(fig)
save(frames, "milling_sim.gif", 160)

# ---------------------------------------------------------------- 3) wire cuts
w = json.loads((DATA / "wire_window_sill.json").read_text())
target = np.asarray(w["target"])
frames = []
allp = np.asarray(w["stages"][0]["stock"]).reshape(-1, 3)
for i, st in enumerate(w["stages"]):
    for show_waste in ((False,) if i == 0 else (True, False)):
        fig = plt.figure(figsize=(7.2, 4.8))
        ax = fig.add_subplot(projection="3d")
        stock = np.asarray(st["stock"])
        ax.add_collection3d(Poly3DCollection(stock, facecolor="#c9b9a0", edgecolor="#6b5b45", linewidths=0.15,
                                             alpha=0.55))
        ax.add_collection3d(Poly3DCollection(target, facecolor="#3b82c4", edgecolor="none", alpha=0.85))
        if show_waste and st["waste"]:
            ax.add_collection3d(Poly3DCollection(np.asarray(st["waste"]), facecolor="#d9534f", edgecolor="none",
                                                 alpha=0.6))
        equal_box(ax, allp, zoom=1.0, min_frac=0.3)
        ax.view_init(elev=28, azim=-62)
        clean(ax)
        fig.subplots_adjust(left=0, right=1, top=0.92, bottom=0.08)
        ax.set_title("Diamond-wire squaring of a window-sill blank: {} ({} of {} cuts)".format(
            st["label"], i, len(w["stages"]) - 1), fontsize=9)
        fig.text(0.5, 0.04, "stock (tan), part (blue), waste piece of the cut (red).  Final: kerf {:.2f} L, waste "
                 "{:.2f} L, left for milling {:.2f} L".format(w["kerf"] / 1e6, w["waste"] / 1e6, w["to_mill"] / 1e6),
                 ha="center", fontsize=8)
        frames.append(grab(fig))
        plt.close(fig)
frames += [frames[-1]] * 4
save(frames, "wire_cuts.gif", 650)

# ---------------------------------------------------------------- 4) robot wire sweep, official robot meshes
# Display: the official visual (COLLADA) meshes with their material colours. Calculation (IK, collision) used the
# official collision meshes in Tesseract when the data was made.
import importlib.util  # noqa: E402
import math  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python" / "src"))
from lithocam import viz  # noqa: E402

CELLS = Path(__file__).resolve().parents[2] / "cells"
spec = importlib.util.spec_from_file_location("make_cells", CELLS / "make_cells.py")
mc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mc)


def rpy_matrix(xyz, rpy):
    x, y, z = (float(s) for s in xyz.split())
    r, p, w = (float(s) for s in rpy.split())
    cr, sr, cp, sp, cw, sw = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    R = np.array([[cw * cp, cw * sp * sr - sw * cr, cw * sp * cr + sw * sr],
                  [sw * cp, sw * sp * sr + cw * cr, sw * sp * cr - cw * sr],
                  [-sp, cp * sr, cp * cr]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (x, y, z)
    return T


def visual_parts(meshdef, cell_mm=4.0):
    """link -> [(decimated triangles in the link frame, rgb)] from the official visual meshes."""
    out = {}
    for ln, (stl, xyz, rpy) in meshdef.items():
        To = rpy_matrix(xyz, rpy)
        parts = []
        for tri, rgb in viz.read_dae(CELLS / "meshes" / mc.visual_path(stl)):
            t = viz.decimate(tri, cell_mm * 1e-3)
            w = t.reshape(-1, 3) @ To[:3, :3].T + To[:3, 3]
            parts.append((w.reshape(-1, 3, 3), rgb))
        out[ln] = parts
    return out


ROBOTS = {"ur10e": (mc.UR_MESH, "UR10e"), "kr120": (mc.KR_MESH, "KUKA KR120 R2500 PRO")}
col = json.loads((DATA / "ruled_twisted_column.json").read_text())
part_mm = np.vstack([np.asarray(f["tris"]) for f in col if f["tris"]])
for rname, (meshdef, label) in ROBOTS.items():
    r = json.loads((DATA / "robot_{}_column.json".format(rname)).read_text())
    off = np.asarray(r["placement"])
    vis = visual_parts(meshdef)
    hits = sum(s["colliding"] for s in r["summary"])
    solved = sum(s["solved"] for s in r["summary"])
    total = sum(s["rulings"] for s in r["summary"])

    def part_at(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        return (part_mm.reshape(-1, 3) * 1e-3) @ Rz.T + off

    table = np.array([[[-0.4, -0.4, 0], [0.4, -0.4, 0], [0.4, 0.4, 0]], [[-0.4, -0.4, 0], [0.4, 0.4, 0], [-0.4, 0.4, 0]]])
    table = table + np.array([off[0], off[1], -0.001])
    pts = [part_at(0.0), table.reshape(-1, 3)]
    for fr in r["frames"][::6]:
        for ln, parts in vis.items():
            T = np.asarray(fr["tf"][ln])
            pts += [p.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3] for p, _ in parts]
    pts = np.vstack(pts)
    frames = []
    for fr in r["frames"]:
        fig = plt.figure(figsize=(6.8, 5.6))
        ax = fig.add_subplot(projection="3d")
        ax.add_collection3d(Poly3DCollection(table, facecolors=viz.shaded(table, (0.55, 0.55, 0.55), 0.5),
                                             edgecolors="none"))
        part = part_at(fr["yaw"]).reshape(-1, 3, 3)
        ax.add_collection3d(Poly3DCollection(part, facecolors=viz.shaded(part, (0.79, 0.73, 0.63), 0.5),
                                             edgecolors="none"))
        for ln, parts in vis.items():
            T = np.asarray(fr["tf"][ln])
            for p, rgb in parts:
                w = p @ T[:3, :3].T + T[:3, 3]
                ax.add_collection3d(Poly3DCollection(w, facecolors=viz.shaded(w, rgb), edgecolors="none"))
        a, b = np.asarray(fr["wire"])
        t0 = np.asarray(fr["tool0"])
        for e in (a, b):
            ax.plot(*zip(t0, e), color="#2d3748", lw=1.6)
        ax.plot(*zip(a, b), color="#d9534f", lw=2.4)
        equal_box(ax, pts, pad=0.0, zoom=1.1, min_frac=0.3)
        ax.view_init(elev=18, azim=-150 if rname == "ur10e" else -120)
        clean(ax)
        fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.07)
        ax.set_title("{} wire sweep, face {}, turntable {:.0f} deg".format(label, fr["face"],
                                                                           math.degrees(fr["yaw"]) % 360), fontsize=9)
        fig.text(0.5, 0.04, "collision-checked with the official collision meshes: {} of {} rulings solved, {} "
                 "colliding configurations".format(solved, total, hits), ha="center", fontsize=7.5)
        frames.append(grab(fig))
        plt.close(fig)
    save(frames, "robot_wire_sweep.gif" if rname == "ur10e" else "robot_wire_sweep_kr120.gif", 220)
