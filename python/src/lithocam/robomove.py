# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Readers for ROBOmove cell backups, and the mapping of ROBOmove tools to lithocam tools.

The data is Quarra Stone's public Q-LAB repository (https://github.com/brnmsmith/Q-LAB, ROBOmove backups of their
KUKA KR 480 MT and KR 500 cells). That repository has no licence file. lithocam therefore never contains a copy of
its files or of their contents: every function here reads them at run time from a local clone, found through the
environment variable LITHOCAM_QLAB or the default `D:/robot_ws/oss/external/Q-LAB`. Tests skip when it is absent.

Formats read
- T_*.ini   tool tables (TOOL_LIST)
- EE_*.ini  end effectors (END_EFFECTOR_LIST)
- RFCONF_*  work-table frames (RFCONF)
- TP_*.ini  ranges of the tool-path parameters (TP_PARAMETERS)
- L_*.ini   robot-program text fragments (P0..P2) and the APT word templates (APT_LAN)
- CM_*.ini  collision models (CM_LIST)
- tcp_spindle.txt
- *.iv      Open Inventor 1.0 ASCII meshes (Coordinate3 + FaceSet)
- *.stl     binary and ASCII STL
- *.ase     3ds Max ASCII export
- *.3ds     3D Studio binary
- *.tga, *.bmp   image headers (pixels are not decoded)
- *.dwp     ROBOmove workpiece/scene file: binary, only partly understood (names, file references)
- *.dll     cell plug-ins: PE header and strings only (the code is not read)

Everything is plain dataclasses and numpy arrays. Units are as stored (ROBOmove tool values and end-effector TCPs
are mm and degrees; .iv files state METERS or MILLIMETERS and are returned in metres).
"""
from __future__ import annotations

import os
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

CREDIT = "Quarra Stone Q-LAB (ROBOmove backup)"
QLAB_URL = "https://github.com/brnmsmith/Q-LAB"
DEFAULT_QLAB = Path("D:/robot_ws/oss/external/Q-LAB")


def qlab_root() -> Optional[Path]:
    """The local Q-LAB clone: $LITHOCAM_QLAB, else the default path; None when it does not exist."""
    p = Path(os.environ.get("LITHOCAM_QLAB") or DEFAULT_QLAB)
    return p if p.is_dir() else None


# ------------------------------------------------------------------------------------------------ tag trees
@dataclass
class Node:
    tag: str
    text: str = ""
    kids: List["Node"] = field(default_factory=list)

    def find(self, tag: str) -> Optional["Node"]:
        for k in self.kids:
            if k.tag == tag:
                return k
        return None

    def get(self, tag: str, default: str = "") -> str:
        k = self.find(tag)
        return k.text if k is not None else default

    def getf(self, tag: str, default: float = 0.0) -> float:
        try:
            return float(self.get(tag, "").split()[0])
        except (ValueError, IndexError):
            return default


_TAG = re.compile(r"<(/?)([^<>\s/]+)>")


def parse_tags(text: str) -> Node:
    """ROBOmove files look like XML but use numbers as tag names (<0>, <1>), so they are not valid XML. This is a
    small tolerant parser: a stack of open tags, text kept as the concatenated text between tags."""
    root = Node("#root")
    stack = [root]
    pos = 0
    for m in _TAG.finditer(text):
        chunk = text[pos:m.start()]
        if chunk.strip():
            stack[-1].text += chunk.strip()
        pos = m.end()
        closing, name = m.group(1) == "/", m.group(2)
        if closing:
            for i in range(len(stack) - 1, 0, -1):
                if stack[i].tag == name:
                    del stack[i:]
                    break
        else:
            n = Node(name)
            stack[-1].kids.append(n)
            stack.append(n)
    return root


def _read_text(path) -> str:
    return Path(path).read_text(encoding="latin-1")


def _tree(path) -> Node:
    t = parse_tags(_read_text(path))
    return t.kids[0] if t.kids and t.kids[0].tag == "ROBOMOVE" else t


def _numbered(node: Optional[Node]) -> List[Node]:
    """Children whose tag is an integer (<0>, <1>, ...), in file order. The declared <SZ> is not trusted: a file
    may hold more entries than its SZ says."""
    return [k for k in (node.kids if node else []) if k.tag.isdigit()]


# ------------------------------------------------------------------------------------------------ tools
# Roles of the ten numbers of a tool's VALUES vector, worked out from the files (docs/ROBOMOVE_IMPORT.md).
# status: "evidence" = the name and the numbers agree across tools; "inferred" = typical for the tool type but not
# stated; "unlabelled" = no meaning could be established.
VALUE_ROLES: List[Tuple[str, str, str]] = [
    ("v0", "cutter diameter (milling); arbor or hub diameter (disc)", "evidence for milling, inferred for disc"),
    ("v1", "holder or flange diameter", "inferred"),
    ("v2", "disc diameter (disc); holder length or a copy of v0/v1 (milling)", "evidence for disc, unlabelled for milling"),
    ("v3", "", "unlabelled"),
    ("v4", "", "unlabelled"),
    ("v5", "", "unlabelled"),
    ("v6", "tool length from the end-effector TCP (milling); offset of the disc plane (disc)", "evidence for milling, unlabelled for disc"),
    ("v7", "disc thickness (disc); 20, 10.7 or 0 on milling tools", "inferred for disc, unlabelled for milling"),
    ("v8", "", "unlabelled"),
    ("v9", "always 0 in every file", "unlabelled"),
]
TOOL_TYPES = {0: "milling (cylindrical or toric)", 1: "round or tapered nose (ball, PCD, taper)", 2: "disc"}


@dataclass
class RoboTool:
    index: int                    # position in the file
    id: int                       # <ID>; not unique in a table
    name: str
    tool_type: int                # <TOOL_TYPE> 0 milling, 1 round nose, 2 disc (see TOOL_TYPES)
    tcp_type: int                 # <TCP_TYPE> 0 or 1: meaning not stated (1 mostly on ball tools)
    spindle_assoc: int            # <SPINDLE_ASSOC>, -1 in every file
    values: Tuple[float, ...]     # the ten numbers of <VALUES>, raw


def read_tools(path) -> List[RoboTool]:
    root = _tree(path)
    out = []
    for n in _numbered(root.find("TOOL_LIST")):
        vals = tuple(float(x) for x in n.get("VALUES").split())
        out.append(RoboTool(len(out), int(n.getf("ID")), n.get("NAME"), int(n.getf("TOOL_TYPE")),
                            int(n.getf("TCP_TYPE")), int(n.getf("SPINDLE_ASSOC", -1)), vals))
    return out


def declared_tool_count(path) -> int:
    n = _tree(path).find("TOOL_LIST")
    return int(n.getf("SZ")) if n else 0


# ------------------------------------------------------------------------------------------------ end effectors
@dataclass
class EndEffector:
    name: str
    file: str                     # path on the ROBOmove PC (not used by lithocam)
    uom: int                      # <UOM>, 0 in every file; unit of the mesh file (meaning not stated)
    tcp: Tuple[float, ...]        # x y z (mm), a b c (deg), then a flag; ROBOmove's own order, see docs
    type: int
    frame_id: int
    spindle_id: int
    visibility: int


def read_end_effectors(path) -> Tuple[int, List[EndEffector]]:
    """(selected index, end effectors)."""
    root = _tree(path)
    out = []
    for n in _numbered(root.find("END_EFFECTOR_LIST")):
        out.append(EndEffector(n.get("NAME"), n.get("FILE"), int(n.getf("UOM")),
                               tuple(float(x) for x in n.get("TCP").split()), int(n.getf("TYPE")),
                               int(n.getf("FRAMEID", -1)), int(n.getf("SPINDLEID", -1)), int(n.getf("VISIBILITY"))))
    return int(root.getf("END_EFFECTOR_SEL")), out


# ------------------------------------------------------------------------------------------------ RFCONF
@dataclass
class TableFrame:
    name: str                     # ROTATING_TABLE, ROTATING_TABLE_2, FIXED_TABLE_1, FIXED_TABLE_2
    xyz: Tuple[float, float, float]            # POSX POSY POSZ, mm
    rpy: Tuple[float, float, float]            # ROLL PITCH YAW, degrees
    euler_config: int             # EULERCONFIG (1 in every file; convention table not stated)
    file_fixed: str
    file_mob: str


def read_rfconf(path) -> Dict[str, TableFrame]:
    root = _tree(path)
    out = {}
    for n in (root.find("RFCONF").kids if root.find("RFCONF") else []):
        out[n.tag] = TableFrame(n.tag, (n.getf("POSX"), n.getf("POSY"), n.getf("POSZ")),
                                (n.getf("ROLL"), n.getf("PITCH"), n.getf("YAW")), int(n.getf("EULERCONFIG")),
                                n.get("FILEFIXED"), n.get("FILEMOB"))
    return out


# ------------------------------------------------------------------------------------------------ TP, L, CM
def read_tp(path) -> Dict[str, Dict[str, float]]:
    """TP_PARAMETERS: {group: {field: value}} for PIECE_COORD, TOOL_COORD and JOINT_COORD (ranges, DELTA_SPINPOS)."""
    root = _tree(path)
    out = {}
    for g in (root.find("TP_PARAMETERS").kids if root.find("TP_PARAMETERS") else []):
        out[g.tag] = {k.tag: float(k.text) for k in g.kids if k.text}
    return out


@dataclass
class ProgramFragments:
    blocks: Dict[str, Tuple[str, List[str]]]               # P0..: (description, lines of KRL text)
    apt: Dict[str, Tuple[str, str, str, str]]               # word: (prefix, postfix, condition, action)


def read_lang(path) -> ProgramFragments:
    root = _tree(path)
    blocks, apt = {}, {}
    for n in root.kids:
        if re.fullmatch(r"P\d+", n.tag):
            blocks[n.tag] = (n.get("DESCR"), [k.text for k in _numbered(n)])
        elif n.tag == "APT_LAN":
            for w in n.kids:
                apt[w.tag] = (w.get("PREFIX"), w.get("POSTFIX"), w.get("CONDITION"), w.get("ACTION"))
    return ProgramFragments(blocks, apt)


@dataclass
class CollisionModel:
    name: str
    visibility: int
    file: str
    uom: int
    pos: Tuple[float, ...]
    dwpname: str


def read_cm(path) -> List[CollisionModel]:
    root = _tree(path)
    return [CollisionModel(n.get("NAME"), int(n.getf("VISIBILITY")), n.get("FILE"), int(n.getf("UOM")),
                           tuple(float(x) for x in n.get("POS").split()), n.get("DWPNAME"))
            for n in _numbered(root.find("CM_LIST"))]


def read_tcp_txt(path) -> Dict[str, float]:
    """tcp_spindle.txt: 'x = 261.5' lines, then a unit remark. Keys lowercased."""
    out = {}
    for ln in _read_text(path).splitlines():
        m = re.match(r"\s*(\w+)\s*=\s*(-?[\d.]+)", ln)
        if m:
            out[m.group(1).lower()] = float(m.group(2))
    return out


# ------------------------------------------------------------------------------------------------ meshes
@dataclass
class Shape:
    vertices: np.ndarray          # (N, 3) metres
    triangles: np.ndarray         # (M, 3) int
    color: Tuple[float, float, float] = (0.7, 0.7, 0.7)
    name: str = ""


def _floats(s: str) -> np.ndarray:
    return np.array(re.findall(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?", s), dtype=float)


def read_iv(path) -> List[Shape]:
    """Open Inventor 1.0 ASCII as written by ROBOmove: Units, Material diffuseColor, Coordinate3 point [...] and
    FaceSet numVertices [...] (faces take consecutive vertices; polygons are fanned). Returned in metres."""
    t = _read_text(path)
    u = re.search(r"units\s+(\w+)", t)
    scale = {"METERS": 1.0, "MILLIMETERS": 1e-3, "CENTIMETERS": 1e-2, "INCHES": 0.0254, "FEET": 0.3048}.get(
        u.group(1).upper() if u else "METERS", 1.0)
    shapes, color, pts = [], (0.7, 0.7, 0.7), None
    pat = re.compile(r"diffuseColor\s+([-\d.eE+ ]+?)\s*[\n}]|Coordinate3\s*\{\s*point\s*\[(.*?)\]|"
                     r"FaceSet\s*\{\s*numVertices\s*\[(.*?)\]", re.S)
    for m in pat.finditer(t):
        if m.group(1):
            c = _floats(m.group(1))
            if len(c) >= 3:
                color = tuple(c[:3])
        elif m.group(2) is not None:
            pts = _floats(m.group(2)).reshape(-1, 3) * scale
        elif m.group(3) is not None and pts is not None:
            nv = _floats(m.group(3)).astype(int)
            tris, i = [], 0
            for n in nv:
                for k in range(1, n - 1):
                    tris.append((i, i + k, i + k + 1))
                i += n
            shapes.append(Shape(pts, np.array(tris, dtype=int).reshape(-1, 3), color, Path(path).stem))
    return shapes


def read_stl(path) -> Shape:
    """Binary or ASCII STL (units as stored; the Q-LAB wall is in mm: scale at the call site)."""
    b = Path(path).read_bytes()
    n = struct.unpack_from("<I", b, 80)[0] if len(b) >= 84 else -1
    if len(b) == 84 + 50 * n:
        a = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
        tri = a["v"].astype(float).reshape(-1, 3)
    else:
        tri = _floats(" ".join(re.findall(r"vertex\s+([^\n]+)", b.decode("latin-1")))).reshape(-1, 3)
    return Shape(tri, np.arange(len(tri)).reshape(-1, 3), name=Path(path).stem)


def read_ase(path) -> List[Shape]:
    """3ds Max ASCII export: one Shape per *GEOMOBJECT (vertex list and face list)."""
    t = _read_text(path)
    out = []
    for g in re.split(r"\*GEOMOBJECT", t)[1:]:
        nm = re.search(r'\*NODE_NAME\s+"([^"]*)"', g)
        v = re.findall(r"\*MESH_VERTEX\s+\d+\s+(\S+)\s+(\S+)\s+(\S+)", g)
        f = re.findall(r"\*MESH_FACE\s+\d+:\s+A:\s*(\d+)\s+B:\s*(\d+)\s+C:\s*(\d+)", g)
        if v and f:
            out.append(Shape(np.array(v, dtype=float), np.array(f, dtype=int), name=nm.group(1) if nm else ""))
    return out


def read_3ds(path) -> List[Shape]:
    """3D Studio binary: walks the chunk tree for object (0x4000) > triangle mesh (0x4100) > vertices (0x4110) and
    faces (0x4120). Materials, textures and transforms are ignored."""
    b = Path(path).read_bytes()
    out = []

    def walk(pos, end, name):
        verts = faces = None
        while pos + 6 <= end:
            cid, ln = struct.unpack_from("<HI", b, pos)
            if ln < 6:
                break
            body, stop = pos + 6, min(pos + ln, end)
            if cid in (0x4D4D, 0x3D3D, 0x4100):
                r = walk(body, stop, name)
                if cid == 0x4100 and r:
                    verts, faces = r
            elif cid == 0x4000:
                z = b.index(b"\x00", body)
                walk(z + 1, stop, b[body:z].decode("latin-1"))
            elif cid == 0x4110:
                n = struct.unpack_from("<H", b, body)[0]
                verts = np.frombuffer(b, "<f4", count=3 * n, offset=body + 2).astype(float).reshape(-1, 3)
            elif cid == 0x4120:
                n = struct.unpack_from("<H", b, body)[0]
                faces = np.frombuffer(b, "<u2", count=4 * n, offset=body + 2).reshape(-1, 4)[:, :3].astype(int)
            pos += ln
        if verts is not None and faces is not None:
            out.append(Shape(verts, faces, name=name))
            return verts, faces
        return None

    walk(0, len(b), "")
    return out


@dataclass
class ImageInfo:
    width: int
    height: int
    bits: int
    kind: str


def read_image_info(path) -> ImageInfo:
    """Header of a TGA or BMP file; pixels are not decoded."""
    b = Path(path).read_bytes()[:64]
    if b[:2] == b"BM":
        if struct.unpack_from("<I", b, 14)[0] == 12:   # OS/2 core header: 16-bit fields
            w, h, _, bits = struct.unpack_from("<HHHH", b, 18)
            return ImageInfo(w, h, bits, "bmp (OS/2 core header)")
        w, h, _, bits = struct.unpack_from("<iiHH", b, 18)
        return ImageInfo(w, abs(h), bits, "bmp")
    w, h, bits = struct.unpack_from("<HHB", b, 12)
    return ImageInfo(w, h, bits, "tga type {}".format(b[2]))


@dataclass
class BinaryInfo:
    kind: str
    detail: str
    strings: List[str]


def _strings(b: bytes, n: int = 6, limit: int = 40) -> List[str]:
    out = [m.group().decode("latin-1") for m in re.finditer(rb"[\x20-\x7e]{%d,}" % n, b)]
    return out[:limit]


def read_dll_info(path) -> BinaryInfo:
    """PE header only: machine, section count, build time stamp, and the first printable strings."""
    b = Path(path).read_bytes()
    if b[:2] != b"MZ":
        return BinaryInfo("not PE", "", [])
    pe = struct.unpack_from("<I", b, 0x3C)[0]
    machine, nsec, stamp = struct.unpack_from("<HHI", b, pe + 4)
    return BinaryInfo("PE", "machine 0x{:x}, {} sections, time stamp {}".format(machine, nsec, stamp), _strings(b))


def read_dwp(path) -> BinaryInfo:
    """ROBOmove .dwp: binary and only partly understood. Returns the referenced mesh files and object names (the
    printable strings). The poses stored in the file are not decoded."""
    b = Path(path).read_bytes()
    s = _strings(b, 4, 400)
    refs = sorted({x for x in s if re.search(r"\.(iv|3ds|ase|stl)$", x, re.I)})
    return BinaryInfo("dwp", "{} bytes, {} mesh references; poses not decoded".format(len(b), len(refs)), refs + s)


# ------------------------------------------------------------------------------------------------ a whole backup
@dataclass
class CellBackup:
    folder: Path
    name: str                                  # e.g. KR500_QUARRASTONE_00
    tools: List[RoboTool] = field(default_factory=list)
    tools_declared: int = 0
    end_effectors: List[EndEffector] = field(default_factory=list)
    tables: Dict[str, TableFrame] = field(default_factory=dict)
    tp: Dict[str, Dict[str, float]] = field(default_factory=dict)
    lang: Optional[ProgramFragments] = None
    collision: List[CollisionModel] = field(default_factory=list)
    tcp_txt: Dict[str, float] = field(default_factory=dict)

    @property
    def robot(self) -> str:
        return self.name.split("_")[0]


def read_cell_backup(folder) -> CellBackup:
    """Reads every ini and text file of one backup folder that is present."""
    folder = Path(folder)
    t = next(iter(sorted(folder.glob("T_*.ini"))), None)
    name = t.stem[2:] if t else folder.name
    cb = CellBackup(folder, name)
    if t:
        cb.tools, cb.tools_declared = read_tools(t), declared_tool_count(t)
    for p in folder.glob("EE_*.ini"):
        cb.end_effectors = read_end_effectors(p)[1]
    for p in folder.glob("RFCONF_*.ini"):
        cb.tables = read_rfconf(p)
    for p in folder.glob("TP_*.ini"):
        cb.tp = read_tp(p)
    for p in folder.glob("L_*.ini"):
        cb.lang = read_lang(p)
    for p in folder.glob("CM_*.ini"):
        cb.collision = read_cm(p)
    p = folder / "spindles" / "tcp_spindle.txt"
    if p.is_file():
        cb.tcp_txt = read_tcp_txt(p)
    return cb


def find_backups(root=None) -> List[Path]:
    """Backup folders under the clone: every folder that holds a T_*.ini or an EE_*.ini."""
    root = Path(root) if root else qlab_root()
    if root is None:
        return []
    seen = sorted({p.parent for p in root.rglob("*.ini") if re.match(r"(T|EE)_", p.name) and ".git" not in p.parts})
    return seen


def read_all(root=None) -> List[CellBackup]:
    return [read_cell_backup(f) for f in find_backups(root)]


# ------------------------------------------------------------------------------------------------ inventory
READERS = {".ini": "tag tree", ".txt": "key = value", ".iv": "Inventor ASCII", ".stl": "STL", ".ase": "ASE",
           ".3ds": "3DS", ".tga": "image header", ".bmp": "image header", ".dwp": "partial (strings)",
           ".dll": "PE header and strings only", ".md": "text"}


def inventory(root=None) -> List[Tuple[str, str, str]]:
    """(relative path, status, note) for every file of the clone outside .git: 'parsed', 'partial' or 'unread'."""
    root = Path(root) if root else qlab_root()
    out = []
    if root is None:
        return out
    for p in sorted(root.rglob("*")):
        if not p.is_file() or ".git" in p.parts:
            continue
        rel, ext = str(p.relative_to(root)).replace("\\", "/"), p.suffix.lower()
        try:
            if ext == ".ini":
                tag = _tree(p).tag
                st, note = "parsed", "root <{}>".format(tag)
            elif ext == ".iv":
                sh = read_iv(p)
                st, note = "parsed", "{} shapes, {} triangles".format(len(sh), sum(len(s.triangles) for s in sh))
            elif ext == ".stl":
                s = read_stl(p)
                st, note = "parsed", "{} triangles".format(len(s.triangles))
            elif ext == ".ase":
                sh = read_ase(p)
                st, note = "parsed", "{} objects, {} triangles".format(len(sh), sum(len(s.triangles) for s in sh))
            elif ext == ".3ds":
                sh = read_3ds(p)
                st, note = "parsed", "{} objects, {} triangles".format(len(sh), sum(len(s.triangles) for s in sh))
            elif ext in (".tga", ".bmp"):
                i = read_image_info(p)
                st, note = "parsed", "{}x{} {} bit".format(i.width, i.height, i.bits)
            elif ext == ".txt":
                st, note = "parsed", "{} keys".format(len(read_tcp_txt(p)))
            elif ext == ".dwp":
                st, note = "partial", read_dwp(p).detail
            elif ext == ".dll":
                st, note = "partial", read_dll_info(p).detail
            elif ext == ".md":
                st, note = "parsed", "text"
            else:
                st, note = "unread", "no reader for {}".format(ext)
        except Exception as e:   # report, do not hide
            st, note = "unread", "{}: {}".format(type(e).__name__, e)
        out.append((rel, st, note))
    return out


# ------------------------------------------------------------------------------------------------ mapping
@dataclass
class ToolMapping:
    tool: RoboTool
    lithocam_id: str = ""
    kind: str = ""
    skipped: str = ""                                  # why no lithocam tool was made
    unmapped: List[Tuple[str, float, str]] = field(default_factory=list)   # (field, value, reason)
    notes: List[str] = field(default_factory=list)
    problems: str = ""                                 # lc.validate_tool message


def _kind_of(t: RoboTool, notes: List[str]) -> str:
    n = t.name.upper()
    if t.tool_type == 2 or any(w in n for w in ("DISK", "DISC", "BLADE", "SAW")):
        if t.tool_type != 2:
            notes.append("TOOL_TYPE {} but the name says a disc: mapped as disc saw".format(t.tool_type))
        return "disc_saw"
    if "BALL" in n:
        if t.tool_type != 1:
            notes.append("TOOL_TYPE {} but the name says ball".format(t.tool_type))
        return "ball_end_mill"
    if "CONIC" in n or "TAPER" in n or "TORIC" in n:
        # The corner radius and the taper angle are not in the file (unlabelled), and lithocam rejects a bull-nose
        # or tapered-ball tool without them. A flat cutter of the same diameter sits at or below the real tip, so
        # a path computed with it does not gouge.
        notes.append("name says {}: corner radius and taper angle are not in the file, so mapped as a flat end mill "
                     "of the same diameter (conservative for gouging)".format(
                         "toric" if "TORIC" in n else "tapered"))
        return "flat_end_mill"
    if t.tool_type == 1:
        notes.append("TOOL_TYPE 1 with no shape in the name: assumed a ball nose")
        return "ball_end_mill"
    return "flat_end_mill"


def to_lithocam_tools(tools: Sequence[RoboTool], prefix: str = "QL", source: str = CREDIT, lc=None):
    """(list of lc.Tool, list of ToolMapping). One mapping per input tool, in order; tools that cannot be mapped (a
    zero-size 'NONE' entry) have `skipped` set and no lc.Tool. Nothing is guessed silently: every value that was not
    used is listed in `unmapped` with the reason, and every judgement call is in `notes`."""
    if lc is None:
        from lithocam.studio.core import lc as _lc
        lc = _lc
    out, maps, used = [], [], {}
    for t in tools:
        m = ToolMapping(t)
        maps.append(m)
        v = list(t.values) + [0.0] * (10 - len(t.values))
        if len(t.values) != 10:
            m.notes.append("VALUES has {} numbers, not 10".format(len(t.values)))
        if v[0] <= 0 and v[2] <= 0:
            m.skipped = "zero-size entry"
            continue
        kind = _kind_of(t, m.notes)
        m.kind = kind
        base = "{}{}".format(prefix, t.id)
        used[base] = used.get(base, 0) + 1
        m.lithocam_id = base if used[base] == 1 else "{}.{}".format(base, used[base])
        if used[base] > 1:
            m.notes.append("ROBOmove ID {} repeats in this table: lithocam id gets a suffix".format(t.id))
        lt = lc.Tool()
        lt.id, lt.name, lt.source = m.lithocam_id, t.name, source
        lt.kind = lc.tool_kind_from_name(kind)
        n = t.name.upper()
        if "PCD" in n:
            lt.material = "PCD"
        elif "CARBIDE" in n:
            lt.material = "carbide"
        if kind == "disc_saw":
            lt.diameter = v[2]
            lt.bore_diameter = v[0]
            lt.holder_diameter = v[1]
            lt.kerf = v[7]
            m.notes.append("disc: diameter from v2 (matches the name, e.g. 640mm DISK -> 642); v0 -> arbor bore and "
                           "v1 -> flange diameter, v7 -> kerf are inferred from magnitude, not stated")
            if v[2] <= v[1]:
                m.notes.append("disc diameter {} is not larger than the flange {}".format(v[2], v[1]))
            m.unmapped.append(("v6", v[6], "disc-plane offset, meaning not stated"))
            for i in (3, 4, 5, 8, 9):
                if v[i]:
                    m.unmapped.append(("v%d" % i, v[i], "unlabelled"))
        else:
            lt.diameter = v[0]
            lt.stickout = v[6]
            if v[1] > v[0]:
                lt.holder_diameter = v[1]
            elif v[1]:
                m.unmapped.append(("v1", v[1], "equals v0: holder not distinguishable"))
            if kind == "ball_end_mill":
                lt.corner_radius = v[0] / 2
                m.notes.append("ball: corner radius = diameter / 2")
            m.unmapped.append(("v2", v[2], "holder length or a copy of v0/v1: unlabelled"))
            for i in (3, 4, 5, 7, 8, 9):
                if v[i]:
                    m.unmapped.append(("v%d" % i, v[i], "unlabelled"))
            if v[7] and v[2] > 400:
                m.notes.append("values 1, 2 and 7 look inherited from a disc template (record not edited)")
            if re.match(r"\d+(\.\d+)?\s*mm", t.name, re.I):
                nominal = float(re.match(r"(\d+(\.\d+)?)", t.name).group(1))
                if abs(nominal - v[0]) > 0.6:
                    m.notes.append("name says {} mm but v0 is {}: the name or the record is stale".format(nominal, v[0]))
        if t.tcp_type:
            m.unmapped.append(("TCP_TYPE", t.tcp_type, "meaning not stated"))
        if t.spindle_assoc != -1:
            m.unmapped.append(("SPINDLE_ASSOC", t.spindle_assoc, "no lithocam field"))
        m.problems = lc.validate_tool(lt)
        out.append(lt)
    return out, maps


# ------------------------------------------------------------------------------------------------ spindle from an EE
def spindle_end_effector(cb: CellBackup) -> Optional[EndEffector]:
    """The end effector of type 1 named 'spindle' (the selected one in every Q-LAB file)."""
    for e in cb.end_effectors:
        if e.type == 1 and "spindle" in e.name.lower():
            return e
    return None


def rpy_matrix(a: float, b: float, c: float) -> np.ndarray:
    """Rotation Rz(a) Ry(b) Rx(c), degrees (KUKA A, B, C order). In every Q-LAB end effector only b is non-zero, so
    the result does not depend on whether ROBOmove reads roll-pitch-yaw as this or the reverse order."""
    a, b, c = np.radians([a, b, c])
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(c), -np.sin(c)], [0, np.sin(c), np.cos(c)]])
    return Rz @ Ry @ Rx
