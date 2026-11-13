# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Qt3D view of the cell: robot (official visual meshes, decimated), part, stock box, toolpath, tool, frames.

Mouse: left drag rotates about the view centre, right or middle drag pans, wheel zooms.
World units are metres, z up. Parts are given in mm in their work frame.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtCore import QByteArray, QPoint, Qt
from PySide6.QtGui import QColor, QMatrix4x4, QVector3D

from .. import viz
from .gizmo import Camera, Gizmo, ring_points
from .kinematics import Robot


# ---------------------------------------------------------------- geometry helpers
def box_tris(size: Sequence[float]) -> np.ndarray:
    sx, sy, sz = (s / 2 for s in size)
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4),
         (1, 5, 7), (1, 7, 3)]
    return c[np.array(f)]


def cylinder_tris(r: float, length: float, segs: int = 24, z0: Optional[float] = None) -> np.ndarray:
    """Cylinder along z, from z0 to z0 + length (centred when z0 is None)."""
    lo = -length / 2 if z0 is None else z0
    hi = lo + length
    a = np.linspace(0, 2 * math.pi, segs + 1)[:-1]
    ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    t = []
    for i in range(segs):
        j = (i + 1) % segs
        p0, p1 = ring[i], ring[j]
        t += [((*p0, lo), (*p1, lo), (*p1, hi)), ((*p0, lo), (*p1, hi), (*p0, hi)),
              ((0, 0, hi), (*p0, hi), (*p1, hi)), ((0, 0, lo), (*p1, lo), (*p0, lo))]
    return np.array(t, dtype=float)


def apply(T: np.ndarray, tris: np.ndarray) -> np.ndarray:
    return (tris.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]).reshape(-1, 3, 3)


def qmatrix(T: np.ndarray) -> QMatrix4x4:
    return QMatrix4x4(*[float(v) for v in np.asarray(T, dtype=float).reshape(-1)])


def link_meshes(robot: Robot, cell: float = 0.003) -> Dict[str, List[Tuple[np.ndarray, Tuple[float, float, float]]]]:
    """Per link: [(triangles in the link frame, rgb)] from the URDF visuals; meshes decimated to `cell` metres."""
    out = {}
    for link, visuals in robot.visuals.items():
        parts = []
        for v in visuals:
            try:
                if v.mesh:
                    if v.mesh.lower().endswith(".dae"):
                        loaded = viz.read_dae(v.mesh)
                    else:
                        loaded = [(viz.read_stl(v.mesh) * np.asarray(v.scale), (0.7, 0.7, 0.72))]
                    for tris, rgb in loaded:
                        parts.append((apply(v.origin, viz.decimate(tris, cell)), rgb))
                elif v.box:
                    parts.append((apply(v.origin, box_tris(v.box)), (0.72, 0.74, 0.78)))
                elif v.cylinder:
                    parts.append((apply(v.origin, cylinder_tris(*v.cylinder)), (0.2, 0.2, 0.22)))
            except Exception:   # noqa: BLE001 - a missing visual must not stop the view
                continue
        if parts:
            out[link] = parts
    return out


# ---------------------------------------------------------------- Qt3D entities
def _buffer(parent, data: np.ndarray):
    buf = Qt3DCore.QBuffer(parent)
    buf.setData(QByteArray(np.ascontiguousarray(data, dtype=np.float32).tobytes()))
    return buf


def _attr(geom, buf, name, size, stride, offset, count):
    a = Qt3DCore.QAttribute(geom)
    a.setName(name)
    a.setVertexBaseType(Qt3DCore.QAttribute.Float)
    a.setVertexSize(size)
    a.setAttributeType(Qt3DCore.QAttribute.VertexAttribute)
    a.setBuffer(buf)
    a.setByteStride(stride)
    a.setByteOffset(offset)
    a.setCount(count)
    geom.addAttribute(a)
    return a


def mesh_entity(parent, tris: np.ndarray, rgb, alpha: float = 1.0):
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    data = np.concatenate([tris.reshape(-1, 3), np.repeat(n, 3, axis=0)], axis=1)
    ent = Qt3DCore.QEntity(parent)
    geom = Qt3DCore.QGeometry(ent)
    buf = _buffer(geom, data)
    cnt = len(data)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultPositionAttributeName(), 3, 24, 0, cnt)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultNormalAttributeName(), 3, 24, 12, cnt)
    rend = Qt3DRender.QGeometryRenderer(ent)
    rend.setGeometry(geom)
    rend.setPrimitiveType(Qt3DRender.QGeometryRenderer.Triangles)
    if alpha < 1.0:
        mat = Qt3DExtras.QPhongAlphaMaterial(ent)
        mat.setAlpha(alpha)
    else:
        mat = Qt3DExtras.QPhongMaterial(ent)
    c = QColor.fromRgbF(*rgb)
    mat.setDiffuse(c)
    mat.setAmbient(c.darker(260))
    mat.setSpecular(QColor(40, 40, 40))
    tf = Qt3DCore.QTransform(ent)
    for comp in (rend, mat, tf):
        ent.addComponent(comp)
    return ent, tf


def colored_mesh_entity(parent, tris: np.ndarray, colors: np.ndarray):
    """Triangles (m, 3, 3) with a colour per triangle corner (m, 3, 3) or per triangle (m, 3), 0..1; flat normals,
    lit by the per-vertex-colour material."""
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    col = np.asarray(colors, dtype=float)
    col = np.repeat(col.reshape(-1, 1, 3), 3, axis=1) if col.ndim == 2 else col.reshape(-1, 3, 3)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    data = np.concatenate([tris.reshape(-1, 3), np.repeat(n, 3, axis=0), col.reshape(-1, 3)], axis=1)
    ent = Qt3DCore.QEntity(parent)
    geom = Qt3DCore.QGeometry(ent)
    buf = _buffer(geom, data)
    cnt = len(data)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultPositionAttributeName(), 3, 36, 0, cnt)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultNormalAttributeName(), 3, 36, 12, cnt)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultColorAttributeName(), 3, 36, 24, cnt)
    rend = Qt3DRender.QGeometryRenderer(ent)
    rend.setGeometry(geom)
    rend.setPrimitiveType(Qt3DRender.QGeometryRenderer.Triangles)
    mat = Qt3DExtras.QPerVertexColorMaterial(ent)
    tf = Qt3DCore.QTransform(ent)
    for comp in (rend, mat, tf):
        ent.addComponent(comp)
    return ent, tf


def lines_entity(parent, segments: np.ndarray, colors: np.ndarray):
    """segments (m, 2, 3); colors (m, 3) 0..1."""
    seg = np.asarray(segments, dtype=float).reshape(-1, 3)
    col = np.repeat(np.asarray(colors, dtype=float).reshape(-1, 3), 2, axis=0)
    nrm = np.tile([0.0, 0.0, 1.0], (len(seg), 1))   # the per-vertex-colour shader also reads a normal
    data = np.concatenate([seg, nrm, col], axis=1)
    ent = Qt3DCore.QEntity(parent)
    geom = Qt3DCore.QGeometry(ent)
    buf = _buffer(geom, data)
    cnt = len(data)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultPositionAttributeName(), 3, 36, 0, cnt)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultNormalAttributeName(), 3, 36, 12, cnt)
    _attr(geom, buf, Qt3DCore.QAttribute.defaultColorAttributeName(), 3, 36, 24, cnt)
    rend = Qt3DRender.QGeometryRenderer(ent)
    rend.setGeometry(geom)
    rend.setPrimitiveType(Qt3DRender.QGeometryRenderer.Lines)
    mat = Qt3DExtras.QPerVertexColorMaterial(ent)
    tf = Qt3DCore.QTransform(ent)
    for comp in (rend, mat, tf):
        ent.addComponent(comp)
    return ent, tf


def frame_segments(T: np.ndarray, size: float):
    o = T[:3, 3]
    segs = np.array([[o, o + T[:3, 0] * size], [o, o + T[:3, 1] * size], [o, o + T[:3, 2] * size]])
    return segs, np.array([[0.9, 0.2, 0.2], [0.2, 0.8, 0.3], [0.25, 0.45, 1.0]])


class CellWindow(Qt3DExtras.Qt3DWindow):
    def __init__(self):
        super().__init__()
        self.defaultFrameGraph().setClearColor(QColor("#262a31"))
        self.root = Qt3DCore.QEntity()
        self.setRootEntity(self.root)
        cam = self.camera()
        cam.lens().setPerspectiveProjection(40.0, 16 / 9, 0.01, 200.0)
        cam.setUpVector(QVector3D(0, 0, 1))
        self.center = np.array([0.5, 0.0, 0.4])
        self.yaw, self.pitch, self.dist = math.radians(-50), math.radians(25), 3.2
        for d in ((-1, -0.6, -1.5), (1, 0.8, -0.6)):
            le = Qt3DCore.QEntity(self.root)
            light = Qt3DRender.QDirectionalLight(le)
            light.setWorldDirection(QVector3D(*d))
            light.setIntensity(0.75)
            le.addComponent(light)
        self._last: Optional[QPoint] = None
        self._button = None
        self.gizmo = Gizmo()
        self.gizmo_on = False
        self.on_gizmo = None             # callback(frame 4x4, final: bool)
        self.on_gizmo_hover = None       # callback(mode or None)
        self.on_camera = None            # callback() after the camera moves
        self.snap = (0.005, math.radians(5))
        self._dragging = False
        self._hover = None
        self.update_camera()

    def camera_model(self) -> Camera:
        cam = self.camera()
        p, c = cam.position(), cam.viewCenter()
        return Camera(np.array([p.x(), p.y(), p.z()]), np.array([c.x(), c.y(), c.z()]), np.array([0, 0, 1.0]),
                      40.0, max(self.width(), 1), max(self.height(), 1))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.camera().lens().setPerspectiveProjection(40.0, max(self.width(), 1) / max(self.height(), 1), 0.01,
                                                      200.0)
        if self.on_camera:
            self.on_camera()

    # camera
    def update_camera(self):
        cp = math.cos(self.pitch)
        eye = self.center + self.dist * np.array([cp * math.cos(self.yaw), cp * math.sin(self.yaw), math.sin(self.pitch)])
        cam = self.camera()
        cam.setPosition(QVector3D(*eye))
        cam.setViewCenter(QVector3D(*self.center))
        cam.setUpVector(QVector3D(0, 0, 1))
        if getattr(self, "on_camera", None):
            self.on_camera()

    def view(self, which: str, bbox: Optional[Tuple[np.ndarray, np.ndarray]] = None):
        if bbox is not None:
            lo, hi = bbox
            self.center = (lo + hi) / 2
            self.dist = max(0.5, 1.6 * float(np.linalg.norm(hi - lo)))
        if which == "iso":
            self.yaw, self.pitch = math.radians(-50), math.radians(25)
        elif which == "top":
            self.yaw, self.pitch = math.radians(-90), math.radians(89)
        elif which == "front":
            self.yaw, self.pitch = math.radians(-90), 0.0
        elif which == "right":
            self.yaw, self.pitch = 0.0, 0.0
        self.update_camera()

    def mousePressEvent(self, e):
        self._last, self._button = e.position().toPoint(), e.button()
        if e.button() == Qt.LeftButton and self.gizmo_on:
            shift = bool(e.modifiers() & Qt.ShiftModifier)
            self.gizmo.snap_m, self.gizmo.snap_rad = (self.snap if shift else (0.0, 0.0))
            pos = e.position()
            if self.gizmo.begin(self.camera_model(), pos.x(), pos.y()):
                self._dragging = True

    def mouseReleaseEvent(self, e):
        self._last = None
        if self._dragging:
            self._dragging = False
            if self.on_gizmo:
                self.on_gizmo(self.gizmo.frame.copy(), True)
            self.gizmo.end()

    def mouseMoveEvent(self, e):
        if self._dragging:
            pos = e.position()
            F = self.gizmo.drag(self.camera_model(), pos.x(), pos.y())
            if F is not None and self.on_gizmo:
                self.on_gizmo(F, False)
            return
        if self._last is None:
            if self.gizmo_on:
                pos = e.position()
                h = self.gizmo.pick(self.camera_model(), pos.x(), pos.y())
                if h != self._hover:
                    self._hover = h
                    if self.on_gizmo_hover:
                        self.on_gizmo_hover(h)
            return
        p = e.position().toPoint()
        dx, dy = p.x() - self._last.x(), p.y() - self._last.y()
        self._last = p
        if self._button == Qt.LeftButton:
            self.yaw -= dx * 0.008
            self.pitch = max(-1.5, min(1.5, self.pitch + dy * 0.008))
        else:
            right = np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])
            up = np.cross(right, [math.cos(self.yaw), math.sin(self.yaw), 0.0])
            k = self.dist * 0.0015
            self.center = self.center - right * dx * k + np.array([0, 0, 1.0]) * dy * k + up * 0
        self.update_camera()

    def wheelEvent(self, e):
        self.dist = max(0.05, self.dist * (0.88 if e.angleDelta().y() > 0 else 1.13))
        self.update_camera()


class CellView:
    """Owns the entities; the main window calls these setters."""

    def __init__(self):
        self.win = CellWindow()
        self.root = self.win.root
        self.robot_links: Dict[str, Qt3DCore.QTransform] = {}
        self.robot_entities: List = []
        self._ents: Dict[str, List] = {}
        self._gizmo_tf = None
        self.win.on_camera = self._place_gizmo
        self.win.on_gizmo_hover = self.highlight_gizmo
        self._grid()

    def _replace(self, key: str, ents: List):
        for e in self._ents.get(key, []):
            e.setParent(None)
            e.deleteLater()
        self._ents[key] = ents

    def _grid(self, half: float = 2.0, step: float = 0.25):
        segs, cols = [], []
        for v in np.arange(-half, half + 1e-9, step):
            c = (0.36, 0.38, 0.42) if abs(v) > 1e-9 else (0.5, 0.52, 0.56)
            segs += [[[v, -half, 0], [v, half, 0]], [[-half, v, 0], [half, v, 0]]]
            cols += [c, c]
        fs, fc = frame_segments(np.eye(4), 0.3)
        e1, _ = lines_entity(self.root, np.array(segs), np.array(cols))
        e2, _ = lines_entity(self.root, fs, fc)
        self._ents["grid"] = [e1, e2]

    # robot
    def set_robot(self, robot: Robot, meshes):
        for e in self.robot_entities:
            e.setParent(None)
            e.deleteLater()
        self.robot_entities, self.robot_links, self._wire_entities = [], {}, []
        for link, parts in meshes.items():
            holder = Qt3DCore.QEntity(self.root)
            tf = Qt3DCore.QTransform(holder)
            holder.addComponent(tf)
            for tris, rgb in parts:
                mesh_entity(holder, tris, rgb)
            self.robot_links[link] = tf
            self.robot_entities.append(holder)
            if link.startswith("wiresaw"):
                self._wire_entities = getattr(self, "_wire_entities", []) + [holder]
        self.show_wire_tool(False)

    def show_wire_tool(self, on: bool):
        """The diamond-wire bow is part of the cell URDF; show it only while a wire tool is active."""
        for e in getattr(self, "_wire_entities", []):
            e.setEnabled(on)

    def update_robot(self, link_tf: Dict[str, np.ndarray], tcp: Optional[np.ndarray] = None):
        for link, tf in self.robot_links.items():
            if link in link_tf:
                tf.setMatrix(qmatrix(link_tf[link]))
        if tcp is not None:
            if "tcp" not in self._ents:
                segs, cols = frame_segments(np.eye(4), 0.08)
                e, tf = lines_entity(self.root, segs, cols)
                self._ents["tcp"] = [e]
                self._tcp_tf = tf
            self._tcp_tf.setMatrix(qmatrix(tcp))
            if getattr(self, "_tool_tf", None) is not None:
                self._tool_tf.setMatrix(qmatrix(self._tool_flange(tcp)))

    # TCP handle (studio.gizmo): arrows translate, rings rotate, the centre square moves in the screen plane
    AXIS_RGB = ((0.92, 0.25, 0.22), (0.3, 0.8, 0.3), (0.3, 0.5, 1.0))

    def show_gizmo(self, on: bool):
        self.win.gizmo_on = on
        if not on:
            self._replace("gizmo", [])
            self._gizmo_tf = None
            return
        self._draw_gizmo(None)

    def set_gizmo(self, frame: np.ndarray, axes: np.ndarray):
        g = self.win.gizmo
        g.frame = np.asarray(frame, dtype=float).copy()
        g.axes = np.asarray(axes, dtype=float).copy()
        self._place_gizmo()

    def _place_gizmo(self):
        if not self.win.gizmo_on or getattr(self, "_gizmo_tf", None) is None:
            return
        g = self.win.gizmo
        T = g.handle_frame()
        T[:3, :3] = T[:3, :3] * g.length(self.win.camera_model())
        self._gizmo_tf.setMatrix(qmatrix(T))

    def _draw_gizmo(self, hot):
        """Unit-size handle in its own frame; the transform scales it to a constant size on screen."""
        holder = Qt3DCore.QEntity(self.root)
        tf = Qt3DCore.QTransform(holder)
        holder.addComponent(tf)
        Rk = [np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),     # z -> x
              np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),     # z -> y
              np.eye(3)]
        for k in range(3):
            rgb = (1.0, 0.85, 0.2) if hot == ("axis", k) else self.AXIS_RGB[k]
            shaft = cylinder_tris(0.018, 0.62, 10, z0=0.22)
            tip = cylinder_tris(0.05, 0.16, 12, z0=0.84)
            tip[:, :, :2] *= ((tip[:, :, 2:3] - 0.84) / -0.16 + 1.0)    # taper to a cone
            for t in (shaft, tip):
                mesh_entity(holder, (t.reshape(-1, 3) @ Rk[k].T).reshape(-1, 3, 3), rgb)
        segs, cols = [], []
        for k in range(3):
            pts = ring_points(np.eye(4), k, Gizmo.RING, 72)
            rgb = (1.0, 0.85, 0.2) if hot == ("ring", k) else self.AXIS_RGB[k]
            segs += [[a, b] for a, b in zip(pts[:-1], pts[1:])]
            cols += [rgb] * (len(pts) - 1)
        s = 0.09
        sq = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]])
        rgb = (1.0, 0.85, 0.2) if hot == ("plane", -1) else (0.95, 0.95, 0.95)
        segs += [[sq[i], sq[(i + 1) % 4]] for i in range(4)]
        cols += [rgb] * 4
        e2, _ = lines_entity(holder, np.array(segs), np.array(cols))
        self._on_top(holder)
        self._replace("gizmo", [holder])
        self._gizmo_tf = tf
        self._place_gizmo()

    @staticmethod
    def _on_top(entity):
        """Draw an entity over everything (no depth test), so the handle is never hidden inside the tool."""
        for e in [entity] + entity.findChildren(Qt3DCore.QEntity):
            for comp in e.components():
                if isinstance(comp, Qt3DRender.QMaterial) and comp.effect() is not None:
                    for tech in comp.effect().techniques():
                        for rp in tech.renderPasses():
                            dt = Qt3DRender.QDepthTest(rp)
                            dt.setDepthFunction(Qt3DRender.QDepthTest.Always)
                            rp.addRenderState(dt)

    def highlight_gizmo(self, hot):
        if self.win.gizmo_on:
            self._draw_gizmo(hot)

    # spindle and cutter on the flange, drawn in the flange frame
    SPINDLE_RGB = {"plate": (0.35, 0.36, 0.4), "body": (0.78, 0.8, 0.84), "nose": (0.3, 0.3, 0.33),
                   "shank": (0.62, 0.62, 0.66), "flutes": (0.9, 0.68, 0.2)}

    def set_tool(self, spindle, cutter_parts, tcp_offset: np.ndarray):
        """spindle: studio.spindles.Spindle or None; cutter_parts: [(name, D mm, z0 mm from the collet face,
        length mm)]; tcp_offset: cutter tip in the flange frame (m)."""
        self._tool_off = tcp_offset
        if spindle is None:
            self._replace("tool", [])
            self._tool_tf = None
            return
        holder = Qt3DCore.QEntity(self.root)
        tf = Qt3DCore.QTransform(holder)
        holder.addComponent(tf)
        ox, oy = spindle.axis_offset
        for name, d, z0, length in spindle.parts():
            if d > 0 and length > 0:
                t = cylinder_tris(d / 2000, length / 1000, 28, z0=z0 / 1000) + np.array([ox, oy, 0]) / 1000
                mesh_entity(holder, t, self.SPINDLE_RGB[name])
        g = spindle.gauge()
        for name, d, z0, length in cutter_parts:
            if d > 0 and length > 0:
                t = cylinder_tris(d / 2000, length / 1000, 20, z0=(g + z0) / 1000) + np.array([ox, oy, 0]) / 1000
                mesh_entity(holder, t, self.SPINDLE_RGB[name])
        self._replace("tool", [holder])
        self._tool_tf = tf

    def _tool_flange(self, tcp: np.ndarray) -> np.ndarray:
        return tcp @ np.linalg.inv(self._tool_off)

    # part, stock, path
    def set_part(self, tris_mm: Optional[np.ndarray], work: np.ndarray):
        if tris_mm is None:
            self._replace("part", [])
            return
        e, tf = mesh_entity(self.root, tris_mm * 1e-3, (0.82, 0.78, 0.7))
        tf.setMatrix(qmatrix(work))
        e.setEnabled(self._part_on)
        fs, fc = frame_segments(work, 0.12)
        e2, _ = lines_entity(self.root, fs, fc)
        self._replace("part", [e, e2])

    _part_on = True

    def set_table(self, turntable: Optional[dict], work: np.ndarray):
        """A rotary table under the work frame origin (disc with a radial mark that turns with it)."""
        if not turntable:
            self._replace("table", [])
            return
        d, h = float(turntable.get("diameter", 0.8)), float(turntable.get("height", 0.04))
        disc = cylinder_tris(d / 2, h, 48, z0=-h - 0.002)
        e, tf = mesh_entity(self.root, disc, (0.42, 0.44, 0.48))
        tf.setMatrix(qmatrix(work))
        mark = np.array([[[0, 0, 0.001], [d / 2, 0, 0.001]]])
        mark = (mark.reshape(-1, 3) @ work[:3, :3].T + work[:3, 3]).reshape(-1, 2, 3)
        e2, _ = lines_entity(self.root, mark, np.array([[1.0, 0.85, 0.2]]))
        self._replace("table", [e, e2])

    def set_stock(self, stock: Optional[dict], work: np.ndarray):
        if not stock:
            self._replace("stock", [])
            return
        lo = np.array([stock["xmin"], stock["ymin"], stock["zmin"]]) * 1e-3
        hi = np.array([stock["xmax"], stock["ymax"], stock["zmax"]]) * 1e-3
        c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
        segs = np.array([[c[a], c[b]] for a, b in edges])
        segs = (segs.reshape(-1, 3) @ work[:3, :3].T + work[:3, 3]).reshape(-1, 2, 3)
        e, _ = lines_entity(self.root, segs, np.tile([0.95, 0.85, 0.3], (len(segs), 1)))
        e.setEnabled(not self._stock_mesh_on)
        self._replace("stock", [e])

    STOCK_RGB = (0.72, 0.6, 0.44)
    _stock_mesh_on = False

    def set_stock_mesh(self, V_mm: Optional[np.ndarray], F: Optional[np.ndarray], work: np.ndarray,
                       colors: Optional[np.ndarray] = None):
        """The simulated stock (studio.process): vertices in mm in the work frame, triangles F; `colors` (n, 3)
        0..1 per vertex (stress colours) or None for a plain Phong surface. It replaces the stock box outline while
        shown; V_mm None removes it and brings the outline back."""
        if V_mm is None or F is None or len(F) == 0:
            self._replace("stock_mesh", [])
            self._stock_mesh_on = False
        else:
            V = np.asarray(V_mm, dtype=float) * 1e-3
            F = np.asarray(F, dtype=np.int64)
            if colors is None:
                e, tf = mesh_entity(self.root, V[F], self.STOCK_RGB)
            else:
                c = 0.12 + 0.72 * np.clip(np.asarray(colors, dtype=float), 0.0, 1.0)   # unsaturated: shading shows
                e, tf = colored_mesh_entity(self.root, V[F], c[F])
            tf.setMatrix(qmatrix(work))
            self._replace("stock_mesh", [e])
            self._stock_mesh_on = True
        for e in self._ents.get("stock", []):
            e.setEnabled(not self._stock_mesh_on)

    def show_part(self, on: bool):
        """Show or hide the part surface (its frame axes stay); kept across set_part calls."""
        self._part_on = bool(on)
        ents = self._ents.get("part", [])
        if ents:
            ents[0].setEnabled(on)

    def set_path(self, pts: Optional[np.ndarray], feed: Optional[np.ndarray], ok: Optional[np.ndarray] = None):
        if pts is None or len(pts) < 2:
            self._replace("path", [])
            return
        segs = np.stack([pts[:-1], pts[1:]], 1)
        f = np.asarray(feed[1:], bool)
        cols = np.where(f[:, None], [1.0, 0.55, 0.1], [0.45, 0.6, 0.85])
        if ok is not None:
            bad = ~np.asarray(ok[1:], bool)
            cols[bad] = [1.0, 0.1, 0.1]
        e, _ = lines_entity(self.root, segs, cols)
        e.setEnabled(self._path_on)
        self._replace("path", [e])

    _path_on = True

    def show_path(self, on: bool):
        """Show or hide the toolpath lines; kept across set_path calls."""
        self._path_on = bool(on)
        for e in self._ents.get("path", []):
            e.setEnabled(on)
