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

# ---------------------------------------------------------------- 4) robot wire sweep
r = json.loads((DATA / "robot_ur10e_column.json").read_text())
col = json.loads((DATA / "ruled_twisted_column.json").read_text())
off = np.asarray(r["placement"])
tris = np.vstack([np.asarray(f["tris"]) for f in col if f["tris"]]) * 1e-3 + off
pts = np.vstack([tris.reshape(-1, 3)] + [np.asarray(fr["links"]) for fr in r["frames"]] + [[[0, 0, 0]]])
frames = []
for k, fr in enumerate(r["frames"]):
    fig = plt.figure(figsize=(6.8, 5.6))
    ax = fig.add_subplot(projection="3d")
    ax.add_collection3d(Poly3DCollection(tris, facecolor="#c9b9a0", edgecolor="#8a7a62", linewidths=0.1,
                                         alpha=0.35))
    L = np.asarray(fr["links"])
    ax.plot(L[:, 0], L[:, 1], L[:, 2], color="#2d3748", lw=4, solid_capstyle="round")
    ax.scatter(L[:, 0], L[:, 1], L[:, 2], color="#e8913a", s=18, depthshade=False)
    a, b = np.asarray(fr["wire"])
    t0 = np.asarray(fr["tool0"])
    for e in (a, b):
        ax.plot(*zip(t0, e), color="#4a5568", lw=1.6)
    ax.plot(*zip(a, b), color="#d9534f", lw=2.4)
    equal_box(ax, pts, pad=0.0, zoom=1.35, min_frac=0.3)
    ax.view_init(elev=18, azim=-150)
    clean(ax)
    fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.07)
    ax.set_title("UR10e wire sweep over the twisted column (face {}), verified IK".format(fr["face"]), fontsize=9)
    fig.text(0.5, 0.04, "lithocam.robot.wire_face_path: kerf-offset rulings, roll redundancy, nearest configuration",
             ha="center", fontsize=8)
    frames.append(grab(fig))
    plt.close(fig)
save(frames, "robot_wire_sweep.gif", 220)
