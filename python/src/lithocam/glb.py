# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Triangles from a binary glTF 2.0 file (.glb), with no third-party reader.

Reads the POSITION attribute and the indices of every triangle primitive and applies the node transforms of the
default scene (matrix, or translation / rotation / scale). Textures, normals and materials are ignored. Compressed
meshes (KHR_draco_mesh_compression, EXT_meshopt_compression) raise ValueError: use the uncompressed file.
Units are those of the file (glTF says metres).
"""
from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import List

import numpy as np

_COMP = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def _chunks(data: bytes):
    magic, version, length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2:
        raise ValueError("not a binary glTF 2.0 file")
    off, js, binc = 12, None, b""
    while off < min(length, len(data)):
        n, kind = struct.unpack_from("<II", data, off)
        body = data[off + 8: off + 8 + n]
        if kind == 0x4E4F534A:
            js = json.loads(body.decode("utf-8"))
        elif kind == 0x004E4942:
            binc = body
        off += 8 + n
    if js is None:
        raise ValueError("glb without a JSON chunk")
    return js, binc


def _accessor(js, binc, i) -> np.ndarray:
    a = js["accessors"][i]
    if "bufferView" not in a:
        raise ValueError("sparse or empty accessors are not supported")
    bv = js["bufferViews"][a["bufferView"]]
    if bv.get("buffer", 0) != 0:
        raise ValueError("external buffers are not supported in a .glb")
    dt = np.dtype(_COMP[a["componentType"]])
    nc = _NCOMP[a["type"]]
    start = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    stride = bv.get("byteStride", 0) or dt.itemsize * nc
    cnt = a["count"]
    if stride == dt.itemsize * nc:
        arr = np.frombuffer(binc, dtype=dt, count=cnt * nc, offset=start).reshape(cnt, nc)
    else:
        raw = np.frombuffer(binc, dtype=np.uint8, count=stride * (cnt - 1) + dt.itemsize * nc, offset=start)
        arr = np.stack([raw[k * stride: k * stride + dt.itemsize * nc].view(dt) for k in range(cnt)])
    return arr.astype(np.float64 if dt.kind == "f" else np.int64)


def _local(node) -> np.ndarray:
    if "matrix" in node:
        return np.asarray(node["matrix"], dtype=float).reshape(4, 4).T     # column-major
    T = np.eye(4)
    x, y, z, w = node.get("rotation", [0.0, 0.0, 0.0, 1.0])
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T[:3, :3] = R * np.asarray(node.get("scale", [1.0, 1.0, 1.0]), dtype=float)
    T[:3, 3] = node.get("translation", [0.0, 0.0, 0.0])
    return T


def load_glb(path) -> np.ndarray:
    """(n, 3, 3) triangles of every mesh in the default scene, in the file's units, node transforms applied."""
    js, binc = _chunks(Path(path).read_bytes())
    used = set(js.get("extensionsUsed", []))
    for ext in ("KHR_draco_mesh_compression", "EXT_meshopt_compression"):
        if ext in used:
            raise ValueError("{} is compressed ({}); use the uncompressed .glb".format(Path(path).name, ext))
    out: List[np.ndarray] = []
    nodes = js.get("nodes", [])

    def walk(i, parent):
        n = nodes[i]
        T = parent @ _local(n)
        if "mesh" in n:
            for prim in js["meshes"][n["mesh"]]["primitives"]:
                if prim.get("mode", 4) != 4:
                    continue                                    # points and lines carry no triangles
                P = _accessor(js, binc, prim["attributes"]["POSITION"])
                idx = (_accessor(js, binc, prim["indices"]).ravel() if "indices" in prim
                       else np.arange(len(P)))
                P = P @ T[:3, :3].T + T[:3, 3]
                tri = P[idx.reshape(-1, 3)]
                if np.linalg.det(T[:3, :3]) < 0:
                    tri = tri[:, ::-1]                          # a mirroring node flips the winding
                out.append(tri)
        for c in n.get("children", []):
            walk(c, T)

    scenes = js.get("scenes") or [{"nodes": list(range(len(nodes)))}]
    for r in scenes[js.get("scene", 0)].get("nodes", []):
        walk(r, np.eye(4))
    if not out:
        raise ValueError("no triangles in " + Path(path).name)
    return np.concatenate(out)
