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
import importlib.util  # noqa: E402
from matplotlib.colors import LightSource  # noqa: E402

CELLS = Path(__file__).resolve().parents[2] / "cells"
spec = importlib.util.spec_from_file_location("make_cells", CELLS / "make_cells.py")
mc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mc)


def read_stl(path):
    data = Path(path).read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, l.split()[1:4])) for l in data.decode(errors="ignore").splitlines()
             if l.strip().startswith("vertex")]
        return np.asarray(v).reshape(-1, 3, 3)
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(np.float64)


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


import math  # noqa: E402

LIGHT = LightSource(azdeg=225, altdeg=45)
LDIR = np.array([-0.4, -0.5, 0.77])
LDIR /= np.linalg.norm(LDIR)


def shaded(tris, hex_colour, alpha=1.0):
    """Lambert shading per triangle (two-sided), as an RGBA array."""
    base = np.array(matplotlib.colors.to_rgb(hex_colour))
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1
    k = 0.35 + 0.65 * np.abs((n / ln[:, None]) @ LDIR)
    rgb = np.clip(base[None, :] * k[:, None], 0, 1)
    return np.hstack([rgb, np.full((len(tris), 1), alpha)])
ROBOTS = {"ur10e": (mc.UR_MESH, "#5f87b6", "UR10e"), "kr120": (mc.KR_MESH, "#f08c00", "KUKA KR120 R2500 PRO")}
col = json.loads((DATA / "ruled_twisted_column.json").read_text())
for rname, (meshdef, colour, label) in ROBOTS.items():
    r = json.loads((DATA / "robot_{}_column.json".format(rname)).read_text())
    off = np.asarray(r["placement"])
    part = np.vstack([np.asarray(f["tris"]) for f in col if f["tris"]]) * 1e-3 + off
    meshes = {ln: (read_stl(CELLS / "meshes" / m[0]), rpy_matrix(m[1], m[2])) for ln, m in meshdef.items()}
    world = []
    for fr in r["frames"][::6]:
        for ln, (tri, To) in meshes.items():
            T = np.asarray(fr["tf"][ln]) @ To
            world.append((tri.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]))
    pts = np.vstack([part.reshape(-1, 3)] + world)
    frames = []
    for k, fr in enumerate(r["frames"]):
        fig = plt.figure(figsize=(6.8, 5.6))
        ax = fig.add_subplot(projection="3d")
        ax.add_collection3d(Poly3DCollection(part, facecolors=shaded(part, "#c9b9a0", 0.45), edgecolors="none"))
        for ln, (tri, To) in meshes.items():
            T = np.asarray(fr["tf"][ln]) @ To
            w = tri @ T[:3, :3].T + T[:3, 3]
            ax.add_collection3d(Poly3DCollection(w, facecolors=shaded(w, colour if ln != "base_link" else "#4a5568"),
                                                 edgecolors="none"))
        a, b = np.asarray(fr["wire"])
        t0 = np.asarray(fr["tool0"])
        for e in (a, b):
            ax.plot(*zip(t0, e), color="#2d3748", lw=1.6)
        ax.plot(*zip(a, b), color="#d9534f", lw=2.4)
        equal_box(ax, pts, pad=0.0, zoom=1.1, min_frac=0.3)
        ax.view_init(elev=18, azim=-150 if rname == "ur10e" else -120)
        clean(ax)
        fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.07)
        ax.set_title("{} wire sweep over the twisted column (face {}), verified IK".format(label, fr["face"]),
                     fontsize=9)
        fig.text(0.5, 0.04, "official robot meshes; lithocam.robot.wire_face_path: kerf-offset rulings, roll "
                 "redundancy, nearest configuration", ha="center", fontsize=7.5)
        frames.append(grab(fig))
        plt.close(fig)
    save(frames, "robot_wire_sweep.gif" if rname == "ur10e" else "robot_wire_sweep_kr120.gif", 220)
