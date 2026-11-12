# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Display helpers: read the robots' visual COLLADA meshes and collision STLs, decimate, shade.

`read_dae` is a minimal COLLADA 1.4 reader for the official robot meshes: positions, <triangles> and <polylist>
(fan-triangulated), material diffuse colours, node <matrix>/<translate>/<rotate>/<scale> transforms, the asset's
unit and up axis. Cameras, lights and anything else are ignored. Display only: calculations use the collision
meshes (STL) in Tesseract.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple

import numpy as np


def _ns(root):
    return root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""


def _floats(text):
    return np.array(text.split(), dtype=np.float64) if text else np.zeros(0)


def _node_matrix(node, ns):
    M = np.eye(4)
    for ch in node:
        tag = ch.tag.replace(ns, "")
        v = _floats(ch.text)
        if tag == "matrix" and v.size == 16:
            M = M @ v.reshape(4, 4)
        elif tag == "translate" and v.size == 3:
            T = np.eye(4)
            T[:3, 3] = v
            M = M @ T
        elif tag == "rotate" and v.size == 4:
            ax, ang = v[:3] / (np.linalg.norm(v[:3]) or 1), math.radians(v[3])
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            R = np.eye(4)
            R[:3, :3] = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
            M = M @ R
        elif tag == "scale" and v.size == 3:
            M = M @ np.diag([v[0], v[1], v[2], 1.0])
    return M


def read_dae(path) -> List[Tuple[np.ndarray, Tuple[float, float, float]]]:
    """[(triangles (N,3,3) in metres, Z up), rgb] per geometry instance and material."""
    root = ET.parse(str(path)).getroot()
    ns = _ns(root)
    unit = root.find("{0}asset/{0}unit".format(ns))
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    up = root.find("{0}asset/{0}up_axis".format(ns))
    up = up.text.strip() if up is not None and up.text else "Z_UP"
    # materials -> effects -> diffuse colour
    eff_col = {}
    for eff in root.iter(ns + "effect"):
        c = eff.find(".//{0}diffuse/{0}color".format(ns))
        if c is not None:
            eff_col[eff.get("id")] = tuple(_floats(c.text)[:3])
    mat_col = {}
    for mat in root.iter(ns + "material"):
        ie = mat.find(ns + "instance_effect")
        if ie is not None:
            mat_col[mat.get("id")] = eff_col.get(ie.get("url", "").lstrip("#"), (0.6, 0.6, 0.6))
    # geometries: id -> [(symbol, triangles in geometry coords)]
    geoms = {}
    for g in root.iter(ns + "geometry"):
        mesh = g.find(ns + "mesh")
        if mesh is None:
            continue
        srcs = {s.get("id"): _floats(s.find(ns + "float_array").text) for s in mesh.findall(ns + "source")
                if s.find(ns + "float_array") is not None}
        verts = mesh.find(ns + "vertices")
        pos_src = None
        if verts is not None:
            for inp in verts.findall(ns + "input"):
                if inp.get("semantic") == "POSITION":
                    pos_src = inp.get("source").lstrip("#")
        if pos_src is None or pos_src not in srcs:
            continue
        P = srcs[pos_src].reshape(-1, 3)
        parts = []
        for prim in list(mesh.findall(ns + "triangles")) + list(mesh.findall(ns + "polylist")):
            inputs = prim.findall(ns + "input")
            stride = max(int(i.get("offset", 0)) for i in inputs) + 1
            voff = next(int(i.get("offset", 0)) for i in inputs if i.get("semantic") == "VERTEX")
            p = prim.find(ns + "p")
            if p is None or not p.text:
                continue
            idx = np.array(p.text.split(), dtype=np.int64).reshape(-1, stride)[:, voff]
            if prim.tag.endswith("triangles"):
                tri = idx.reshape(-1, 3)
            else:
                vc = np.array(prim.find(ns + "vcount").text.split(), dtype=np.int64)
                tris, k = [], 0
                for n in vc:
                    poly = idx[k:k + n]
                    tris += [(poly[0], poly[i], poly[i + 1]) for i in range(1, n - 1)]
                    k += n
                tri = np.array(tris, dtype=np.int64).reshape(-1, 3)
            parts.append((prim.get("material"), P[tri]))
        geoms[g.get("id")] = parts
    up_fix = np.eye(4)
    if up == "Y_UP":
        up_fix[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    out = []

    def walk(node, M):
        M = M @ _node_matrix(node, ns)
        for ig in node.findall(ns + "instance_geometry"):
            bind = {im.get("symbol"): im.get("target", "").lstrip("#") for im in ig.iter(ns + "instance_material")}
            for sym, tri in geoms.get(ig.get("url", "").lstrip("#"), []):
                w = tri.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]
                w = (w @ up_fix[:3, :3].T) * scale
                out.append((w.reshape(-1, 3, 3), mat_col.get(bind.get(sym, ""), (0.6, 0.6, 0.6))))
        for ch in node.findall(ns + "node"):
            walk(ch, M)

    for vs in root.iter(ns + "visual_scene"):
        for n in vs.findall(ns + "node"):
            walk(n, np.eye(4))
    return out


def read_stl(path) -> np.ndarray:
    data = Path(path).read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, l.split()[1:4])) for l in data.decode(errors="ignore").splitlines()
             if l.strip().startswith("vertex")]
        return np.asarray(v).reshape(-1, 3, 3)
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(np.float64)


def decimate(tris: np.ndarray, cell: float) -> np.ndarray:
    """Vertex clustering on a grid of size `cell`: vertices in one cell merge to their mean; degenerate triangles go."""
    v = tris.reshape(-1, 3)
    key = np.floor(v / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((inv.max() + 1, 3))
    np.add.at(sums, inv, v)
    cnt = np.bincount(inv)[:, None]
    rep = sums / cnt
    t = inv.reshape(-1, 3)
    keep = (t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])
    t = np.unique(np.sort(t[keep], axis=1), axis=0) if False else t[keep]
    return rep[t]


LDIR = np.array([-0.4, -0.5, 0.77]) / np.linalg.norm([-0.4, -0.5, 0.77])


def shaded(tris: np.ndarray, rgb, alpha: float = 1.0) -> np.ndarray:
    base = np.asarray(rgb, float)[:3]
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1
    k = 0.35 + 0.65 * np.abs((n / ln[:, None]) @ LDIR)
    return np.hstack([np.clip(base[None, :] * k[:, None], 0, 1), np.full((len(tris), 1), alpha)])
