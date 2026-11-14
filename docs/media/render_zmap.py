# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Frames of a Z-map run (lithocam.studio.process.zmap_stock saved as .npz): per step, the cut surface shaded as a
relief beside the stone left over the part (white 0, dark red at `vmax` mm and above, blue where the cut is below
the part). Needs numpy and Pillow.

    python docs/media/render_zmap.py RUN.npz OUTDIR [x0,y0,x1,y1 mm window] [vmax mm] [width px]
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def shade(H, cell, light=(-1.0, 1.0, 1.6)):
    gy, gx = np.gradient(H, cell)
    n = np.dstack([-gx, -gy, np.ones_like(H)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    L = np.asarray(light, float)
    L /= np.linalg.norm(L)
    v = np.clip(n @ L, 0, 1)
    base = np.array([214, 200, 176], float) / 255.0         # light sandstone tone
    return (np.clip(0.25 + 0.75 * v, 0, 1)[..., None] * base * 255).astype(np.uint8)


def leftover(D, vmax):
    t = np.clip(D / vmax, 0, 1)[..., None]
    up = (1 - t) * np.array([255, 255, 255.0]) + t * np.array([150, 20, 20.0])
    g = np.clip(-D / 0.1, 0, 1)[..., None]                   # gouge: blue at 0.1 mm below the part and more
    out = (1 - g) * up + g * np.array([30, 70, 200.0])
    return out.astype(np.uint8)


def main():
    run, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    z = np.load(run, allow_pickle=False)
    cell, x0, y0 = float(z["cell"]), float(z["x0"]), float(z["y0"])
    T, Hs, names = z["target"], z["heights"], [str(n) for n in z["names"]]
    win = [float(v) for v in sys.argv[3].split(",")] if len(sys.argv) > 3 else None
    vmax = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    width = int(sys.argv[5]) if len(sys.argv) > 5 else 1280
    if win:
        ix = slice(int((win[0] - x0) / cell), int((win[2] - x0) / cell))
        iy = slice(int((win[1] - y0) / cell), int((win[3] - y0) / cell))
    else:
        ix = iy = slice(None)
    for k, (name, H) in enumerate(zip(names, Hs)):
        h, t = H[iy, ix][::-1], T[iy, ix][::-1]            # row 0 at y = y0: flip so +y is up in the image
        a, b = Image.fromarray(shade(h, cell)), Image.fromarray(leftover(h - t, vmax))
        w2 = width // 2
        sz = (w2, int(w2 * a.height / a.width))
        img = Image.new("RGB", (width, sz[1] + 44), (32, 36, 42))
        img.paste(a.resize(sz, Image.LANCZOS), (0, 44))
        img.paste(b.resize(sz, Image.LANCZOS), (w2, 44))
        d = ImageDraw.Draw(img)
        d.text((10, 6), "{}  ({} of {})".format(name, k + 1, len(names)), fill=(235, 235, 235))
        d.text((10, 24), "cut surface, {:g} mm Z-map".format(cell), fill=(190, 190, 190))
        d.text((w2 + 10, 24), "stone left over the part: white 0, red {:g} mm+, blue = below the part".format(vmax),
               fill=(190, 190, 190))
        img.save(out / "frame_{:03d}.png".format(k))
    print("wrote", len(names), "frames to", out)


if __name__ == "__main__":
    main()
