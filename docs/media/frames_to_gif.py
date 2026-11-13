# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Turn studio frames (python -m lithocam.studio --record DIR N ...) into a docs GIF (needs Pillow).

    python docs/media/frames_to_gif.py DIR OUT.gif [width=720] [ms per frame=90] [crop=l,t,r,b fractions]
"""
import sys
from pathlib import Path

from PIL import Image


def main(argv):
    src, out = Path(argv[1]), Path(argv[2])
    width = int(argv[3]) if len(argv) > 3 else 720
    ms = int(argv[4]) if len(argv) > 4 else 90
    crop = [float(v) for v in argv[5].split(",")] if len(argv) > 5 else None
    frames = []
    for f in sorted(src.glob("frame_*.png")):
        im = Image.open(f).convert("RGB")
        if crop:
            w, h = im.size
            im = im.crop((int(crop[0] * w), int(crop[1] * h), int(crop[2] * w), int(crop[3] * h)))
        im.thumbnail((width, width * 2))
        frames.append(im)
    if not frames:
        raise SystemExit("no frame_*.png in {}".format(src))
    pal = frames[len(frames) // 2].convert("P", palette=Image.ADAPTIVE, colors=128)
    seq = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in frames]
    seq += [seq[-1]] * 8                                  # hold the last frame
    seq[0].save(out, save_all=True, append_images=seq[1:], duration=ms, loop=0, optimize=True)
    print(out, len(frames), "frames", out.stat().st_size // 1024, "KB")


if __name__ == "__main__":
    main(sys.argv)
