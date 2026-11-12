# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""lithocam studio: one window for stone CAM and robot programming.

Layout (RoboDK's station tree and robot panel, RhinoCAM's machining browser and operation toolbars):

  toolbars   Station (open, save, part, tools, views, teach, simulate, post) and Machining (setup and one
             drop-down per operation category, generate, stock check)
  left       Work tree: station > robot > tools, work frame > part, stock, programs > steps (drag to reorder,
             drop palette items to insert); below it the Objects browser: Operations palette and Tool library
  centre     3D cell view with a simulation bar
  right      Robot panel (joints, TCP, Cartesian jog in base / tool / work frame, teach) and Properties of the
             selected step (tool drop-down filtered by the operation)
  bottom     Log and posted program

Run:  python -m lithocam.studio
"""
from __future__ import annotations

import math
import sys
import traceback
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from PySide6.QtCore import QMimeData, QObject, QSize, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QAction, QBrush, QColor, QFont, QIcon, QPainter, QPixmap
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox, QDockWidget, QDoubleSpinBox,
                               QFileDialog, QFormLayout, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                               QListWidget, QListWidgetItem, QMainWindow, QMenu, QMessageBox, QPlainTextEdit,
                               QPushButton, QScrollArea, QSlider, QSpinBox, QStyle, QTableWidget, QTableWidgetItem, QTabWidget,
                               QToolBar, QToolBox, QToolButton, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from .. import post
from . import cam, catalog, program, spindles
from .core import lc
from .kinematics import matrix_rpy, rpy_matrix
from .view3d import CellView, link_meshes

MIME = "application/x-lithocam-step"
ROLE = Qt.UserRole


# ---------------------------------------------------------------- icons drawn here (no third-party artwork)
def glyph_icon(text: str, colour: str, size: int = 40, status: str = "done") -> QIcon:
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    c = QColor(colour)
    if status == "planned":
        c = c.lighter(150)
        c.setAlpha(150)
    p.setBrush(c)
    p.setPen(Qt.NoPen)
    p.drawRoundedRect(2, 2, size - 4, size - 4, 8, 8)
    p.setPen(QColor("white"))
    f = QFont("Segoe UI", int(size * (0.36 if len(text) <= 2 else 0.26)))
    f.setBold(True)
    p.setFont(f)
    p.drawText(pm.rect(), Qt.AlignCenter, text)
    if status != "done":   # corner dot: partial = amber, planned = grey
        p.setBrush(QColor("#f2b705") if status == "partial" else QColor("#9aa0a6"))
        p.drawEllipse(size - 13, 3, 9, 9)
    p.end()
    return QIcon(pm)


def step_icon(spec: catalog.StepSpec) -> QIcon:
    return glyph_icon(spec.glyph, catalog.CATEGORY_COLOUR.get(spec.category, "#666"), 40, spec.status)


# ---------------------------------------------------------------- palette (drag source)
class PaletteList(QListWidget):
    def __init__(self, specs: List[catalog.StepSpec]):
        super().__init__()
        self.setViewMode(QListWidget.IconMode)
        self.setIconSize(QSize(36, 36))
        self.setGridSize(QSize(92, 74))
        self.setResizeMode(QListWidget.Adjust)
        self.setWordWrap(True)
        self.setDragEnabled(True)
        self.setMovement(QListWidget.Static)
        for s in specs:
            it = QListWidgetItem(step_icon(s), s.label)
            it.setData(ROLE, s.id)
            it.setToolTip("{} ({}){}{}".format(s.label, s.status, "\ntools: " + s.tool_family if s.tool_family else "",
                                                "\n" + s.doc if s.doc else ""))
            self.addItem(it)

    def mimeData(self, items):
        md = QMimeData()
        md.setData(MIME, ",".join(i.data(ROLE) for i in items).encode())
        return md


# ---------------------------------------------------------------- work tree
class WorkTree(QTreeWidget):
    changed = Signal()
    request_add = Signal(str, object)   # step id, insert-after item (or program item)

    def __init__(self):
        super().__init__()
        self.setHeaderHidden(True)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QAbstractItemView.DragDrop)
        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setIconSize(QSize(20, 20))

    def mimeTypes(self):
        return [MIME, "application/x-qabstractitemmodeldatalist"]

    def _target(self, pos):
        it = self.itemAt(pos)
        while it is not None and it.data(0, ROLE) is None and it.data(0, ROLE + 1) != "program":
            it = it.parent()
        return it

    def dragEnterEvent(self, e):
        e.acceptProposedAction()

    def dragMoveEvent(self, e):
        t = self._target(e.position().toPoint())
        if t is not None:
            e.acceptProposedAction()
        else:
            e.ignore()

    def dropEvent(self, e):
        t = self._target(e.position().toPoint())
        if t is None:
            e.ignore()
            return
        if e.mimeData().hasFormat(MIME):
            for sid in bytes(e.mimeData().data(MIME)).decode().split(","):
                self.request_add.emit(sid, t)
            e.acceptProposedAction()
            return
        # internal move of step items: before the target step, or at the end of a program
        moving = [i for i in self.selectedItems() if isinstance(i.data(0, ROLE), program.Step)]
        if not moving or t in moving:
            e.ignore()
            return
        for it in moving:
            it.parent().removeChild(it)
        if t.data(0, ROLE + 1) == "program":
            for it in moving:
                t.addChild(it)
        else:
            par = t.parent()
            idx = par.indexOfChild(t)
            for k, it in enumerate(moving):
                par.insertChild(idx + k, it)
        e.setDropAction(Qt.IgnoreAction)
        e.accept()
        self.changed.emit()


# ---------------------------------------------------------------- tool library panel
class ToolPanel(QWidget):
    assign = Signal(str)

    def __init__(self):
        super().__init__()
        self.tools: List = []
        v = QVBoxLayout(self)
        f = QGridLayout()
        self.domain = QComboBox()
        self.domain.addItems(["all", "stone", "wood", "both"])
        self.kind = QComboBox()
        self.kind.addItem("all kinds", None)
        for k in lc.all_tool_kinds():
            self.kind.addItem(lc.tool_kind_label(k), k)
        self.op = QComboBox()
        self.op.addItem("all operations", "")
        for o in lc.tool_operation_names():
            self.op.addItem(o, o)
        self.cutter_only = QCheckBox("drop-cutter model only")
        f.addWidget(QLabel("Domain"), 0, 0)
        f.addWidget(self.domain, 0, 1)
        f.addWidget(QLabel("Kind"), 1, 0)
        f.addWidget(self.kind, 1, 1)
        f.addWidget(QLabel("Operation"), 2, 0)
        f.addWidget(self.op, 2, 1)
        f.addWidget(self.cutter_only, 3, 1)
        v.addLayout(f)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["id", "name", "kind", "D mm", "model"])
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        v.addWidget(self.table, 1)
        self.detail = QLabel()
        self.detail.setWordWrap(True)
        self.detail.setTextInteractionFlags(Qt.TextSelectableByMouse)
        v.addWidget(self.detail)
        h = QHBoxLayout()
        self.btn_assign = QPushButton("Use for selected step")
        self.btn_load = QPushButton("Load CSV")
        self.btn_save = QPushButton("Save CSV")
        for b in (self.btn_assign, self.btn_load, self.btn_save):
            h.addWidget(b)
        v.addLayout(h)
        for w in (self.domain, self.kind, self.op):
            w.currentIndexChanged.connect(self.refresh)
        self.cutter_only.toggled.connect(self.refresh)
        self.table.itemSelectionChanged.connect(self._show)
        self.btn_assign.clicked.connect(lambda: self.current() and self.assign.emit(self.current().id))
        self.table.doubleClicked.connect(lambda *_: self.current() and self.assign.emit(self.current().id))

    def set_tools(self, tools):
        self.tools = list(tools)
        self.refresh()

    def visible(self):
        out = []
        dom, kind, op = self.domain.currentText(), self.kind.currentData(), self.op.currentData()
        for t in self.tools:
            d = lc.tool_kind_domain(t.kind)
            if dom != "all" and d != dom and not (d == "both" and dom in ("stone", "wood")):
                continue
            if kind is not None and t.kind != kind:
                continue
            if op and op not in lc.tool_operations(t.kind):
                continue
            if self.cutter_only.isChecked() and not lc.has_cutter(t):
                continue
            out.append(t)
        return out

    def refresh(self):
        vis = self.visible()
        self.table.setRowCount(len(vis))
        for r, t in enumerate(vis):
            model = "yes" if lc.has_cutter(t) else lc.tool_kind_envelope(t.kind)
            for c, val in enumerate([t.id, t.name, lc.tool_kind_name(t.kind), "{:g}".format(t.diameter), model]):
                it = QTableWidgetItem(val)
                it.setData(ROLE, t.id)
                self.table.setItem(r, c, it)

    def current(self):
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        if not rows:
            return None
        tid = self.table.item(rows[0].row(), 0).data(ROLE)
        return next((t for t in self.tools if t.id == tid), None)

    def _show(self):
        t = self.current()
        if t is None:
            return
        bits = ["<b>{}</b> {}".format(t.id, t.name), "{} ({}, {})".format(lc.tool_kind_label(t.kind),
                                                                         lc.tool_kind_domain(t.kind),
                                                                         lc.tool_kind_envelope(t.kind))]
        for k in ("material", "bond", "grit", "profile", "helix", "point", "thread"):
            if getattr(t, k):
                bits.append("{}: {}".format(k, getattr(t, k)))
        for k in ("diameter", "corner_radius", "tip_angle", "taper_angle", "flute_length", "neck_diameter",
                  "pilot_diameter", "bore_diameter", "segment_height", "kerf", "length", "stickout", "spindle_rpm",
                  "feed"):
            v = getattr(t, k)
            if v:
                bits.append("{} {:g}".format(k, v))
        bits.append("operations: " + ", ".join(lc.tool_operations(t.kind)))
        bits.append("drop-cutter: " + ("yes" if lc.has_cutter(t) else "no"))
        bits.append("source: " + t.source)
        self.detail.setText("<br>".join(bits))


# ---------------------------------------------------------------- robot panel
class RobotPanel(QWidget):
    moved = Signal(object)          # joint vector (rad)
    teach = Signal(str)             # "movej" or "movel"
    robot_changed = Signal(str)
    spindle_changed = Signal(str)

    STEPS_MM = [0.1, 1.0, 5.0, 10.0, 50.0, 100.0]
    STEPS_DEG = [0.5, 1.0, 5.0, 15.0, 45.0]

    def __init__(self):
        super().__init__()
        self.robot = None
        self.q = np.zeros(6)
        self.work = np.eye(4)
        self._busy = False
        v = QVBoxLayout(self)
        top = QHBoxLayout()
        self.combo = QComboBox()
        for k, (label, _, _) in program.ROBOTS.items():
            self.combo.addItem(label, k)
        top.addWidget(QLabel("Robot"))
        top.addWidget(self.combo, 1)
        v.addLayout(top)
        self.combo.currentIndexChanged.connect(lambda *_: self.robot_changed.emit(self.combo.currentData()))
        sp = QHBoxLayout()
        self.spindle = QComboBox()
        for k, s in spindles.PRESETS.items():
            self.spindle.addItem(s.name, k)
        self.spindle.currentIndexChanged.connect(lambda *_: self.spindle_changed.emit(self.spindle.currentData()))
        sp.addWidget(QLabel("Spindle"))
        sp.addWidget(self.spindle, 1)
        v.addLayout(sp)
        # joints
        g = QGridLayout()
        self.sliders, self.spins = [], []
        for i in range(6):
            s = QSlider(Qt.Horizontal)
            sp = QDoubleSpinBox()
            sp.setDecimals(2)
            sp.setSuffix(" deg")
            sp.setSingleStep(1.0)
            g.addWidget(QLabel("J{}".format(i + 1)), i, 0)
            g.addWidget(s, i, 1)
            g.addWidget(sp, i, 2)
            s.valueChanged.connect(lambda val, k=i: self._joint_from_slider(k, val))
            sp.valueChanged.connect(lambda val, k=i: self._joint_from_spin(k, val))
            self.sliders.append(s)
            self.spins.append(sp)
        box = QWidget()
        box.setLayout(g)
        v.addWidget(QLabel("<b>Joint jog</b>"))
        v.addWidget(box)
        # TCP
        v.addWidget(QLabel("<b>TCP</b> in the robot base (mm, roll-pitch-yaw deg)"))
        tg = QGridLayout()
        self.pose = []
        for i, n in enumerate(["X", "Y", "Z", "Rx", "Ry", "Rz"]):
            sp = QDoubleSpinBox()
            sp.setRange(-100000, 100000)
            sp.setDecimals(3 if i < 3 else 3)
            tg.addWidget(QLabel(n), i // 3 * 1, (i % 3) * 2)
            tg.addWidget(sp, i // 3 * 1, (i % 3) * 2 + 1)
            self.pose.append(sp)
        go = QPushButton("Move to pose")
        go.clicked.connect(self._go_pose)
        tg.addWidget(go, 2, 0, 1, 6)
        w = QWidget()
        w.setLayout(tg)
        v.addWidget(w)
        # Cartesian jog
        v.addWidget(QLabel("<b>Cartesian jog</b>"))
        cg = QGridLayout()
        self.frame = QComboBox()
        self.frame.addItems(["Base", "Tool", "Work"])
        self.lin_step = QComboBox()
        self.rot_step = QComboBox()
        for s in self.STEPS_MM:
            self.lin_step.addItem("{:g} mm".format(s), s)
        for s in self.STEPS_DEG:
            self.rot_step.addItem("{:g} deg".format(s), s)
        self.lin_step.setCurrentIndex(2)
        self.rot_step.setCurrentIndex(2)
        cg.addWidget(QLabel("Frame"), 0, 0)
        cg.addWidget(self.frame, 0, 1)
        cg.addWidget(self.lin_step, 0, 2)
        cg.addWidget(self.rot_step, 0, 3)
        for r, ax in enumerate(["X", "Y", "Z"]):
            for c, (sign, rot) in enumerate([(-1, False), (1, False), (-1, True), (1, True)]):
                b = QPushButton("{}{}{}".format("-" if sign < 0 else "+", "R" if rot else "", ax.lower() if rot else ax))
                b.setAutoRepeat(True)
                b.setAutoRepeatInterval(120)
                b.clicked.connect(lambda *_, a=r, s=sign, rr=rot: self._jog(a, s, rr))
                cg.addWidget(b, r + 1, c)
        w2 = QWidget()
        w2.setLayout(cg)
        v.addWidget(w2)
        self.info = QLabel()
        self.info.setWordWrap(True)
        v.addWidget(self.info)
        h = QHBoxLayout()
        for text, slot in (("Home", self._home), ("Teach MoveJ", lambda: self.teach.emit("movej")),
                           ("Teach MoveL", lambda: self.teach.emit("movel"))):
            b = QPushButton(text)
            b.clicked.connect(slot)
            h.addWidget(b)
        v.addLayout(h)
        note = QLabel("Jogging uses lithocam's damped least-squares IK without collision checks; programs are "
                      "collision-checked in lithocam.robot (Tesseract).")
        note.setWordWrap(True)
        note.setStyleSheet("color: #888")
        v.addWidget(note)
        v.addStretch(1)

    def set_robot(self, robot, q):
        self.robot = robot
        self._busy = True
        for i in range(6):
            enabled = i < robot.dof
            self.sliders[i].setEnabled(enabled)
            self.spins[i].setEnabled(enabled)
            if enabled:
                lo, hi = math.degrees(robot.lower[i]), math.degrees(robot.upper[i])
                self.sliders[i].setRange(int(lo * 10), int(hi * 10))
                self.spins[i].setRange(lo, hi)
        self._busy = False
        self.set_q(q)

    def set_q(self, q, emit=True):
        self.q = np.asarray(q, dtype=float)
        self._busy = True
        for i, v in enumerate(self.q[:6]):
            self.sliders[i].setValue(int(round(math.degrees(v) * 10)))
            self.spins[i].setValue(math.degrees(v))
        T = self.robot.tcp(self.q)
        rpy = matrix_rpy(T[:3, :3])
        for i, v in enumerate(list(T[:3, 3] * 1000) + [math.degrees(a) for a in rpy]):
            self.pose[i].setValue(v)
        self._busy = False
        w = self.robot.manipulability(self.q)
        self.info.setText("Manipulability {:.4f}{}".format(w, "  (near a singularity)" if w < 2e-3 else ""))
        if emit:
            self.moved.emit(self.q)

    def _joint_from_slider(self, k, val):
        if self._busy:
            return
        q = self.q.copy()
        q[k] = math.radians(val / 10)
        self.set_q(q)

    def _joint_from_spin(self, k, val):
        if self._busy:
            return
        q = self.q.copy()
        q[k] = math.radians(val)
        self.set_q(q)

    def _go_pose(self):
        v = [s.value() for s in self.pose]
        T = np.eye(4)
        T[:3, :3] = rpy_matrix([math.radians(a) for a in v[3:]])
        T[:3, 3] = np.array(v[:3]) / 1000
        q, ok = program.solve_start(self.robot, T, self.q)
        if ok:
            self.set_q(q)
        else:
            self.info.setText("Pose out of reach")

    def _jog(self, axis, sign, rot):
        d = np.zeros(6)
        if rot:
            d[3 + axis] = sign * math.radians(self.rot_step.currentData())
        else:
            d[axis] = sign * self.lin_step.currentData() / 1000
        frame = self.frame.currentText().lower()
        if frame == "work":   # express the step in the work frame's axes, then jog in the base frame
            R = self.work[:3, :3]
            d = np.concatenate([R @ d[:3], R @ d[3:]])
            frame = "base"
        q, ok = self.robot.jog(self.q, d, frame)
        if ok:
            self.set_q(q)
        else:
            self.info.setText("Jog limit: out of reach or at a joint limit")

    def _home(self):
        self.set_q(np.asarray(program.HOME.get(self.combo.currentData(), [0.0] * self.robot.dof)))


# ---------------------------------------------------------------- properties
class Properties(QWidget):
    edited = Signal()
    teach_into = Signal(object, str)   # step, param key

    def __init__(self):
        super().__init__()
        self.form = QFormLayout(self)
        self.step: Optional[program.Step] = None
        self.tools: List = []

    def show_step(self, step: Optional[program.Step]):
        while self.form.rowCount():
            self.form.removeRow(0)
        self.step = step
        if step is None:
            return
        spec = step.spec
        title = QLabel("<b>{}</b> <span style='color:#888'>({}, {})</span>".format(spec.label, spec.category,
                                                                                     spec.status))
        self.form.addRow(title)
        if spec.doc:
            d = QLabel(spec.doc)
            d.setWordWrap(True)
            self.form.addRow(d)
        lab = QLineEdit(step.label)
        lab.setPlaceholderText(spec.label)
        lab.textChanged.connect(lambda t: self._set_label(t))
        self.form.addRow("Name", lab)
        if spec.tool_family and "tool" not in [p[0] for p in spec.params]:
            self._tool_row(spec, "tool", "Tool")
        for key, label, kind, default, unit in spec.params:
            val = step.params.get(key, default)
            if kind == "tool":
                self._tool_row(spec, key, label)
                continue
            if kind == "float":
                w = QDoubleSpinBox()
                w.setRange(-1e6, 1e6)
                w.setDecimals(3)
                w.setValue(float(val))
                w.valueChanged.connect(lambda v, k=key: self._set(k, v))
            elif kind == "int":
                w = QSpinBox()
                w.setRange(-100000, 100000)
                w.setValue(int(val))
                w.valueChanged.connect(lambda v, k=key: self._set(k, v))
            elif kind == "bool":
                w = QCheckBox()
                w.setChecked(bool(val))
                w.toggled.connect(lambda v, k=key: self._set(k, v))
            elif kind.startswith("choice:"):
                w = QComboBox()
                w.addItems(kind[7:].split("|"))
                w.setCurrentText(str(val))
                w.currentTextChanged.connect(lambda v, k=key: self._set(k, v))
            elif kind in ("joints", "pose"):
                w = QWidget()
                h = QHBoxLayout(w)
                h.setContentsMargins(0, 0, 0, 0)
                txt = "not set"
                if val is not None and kind == "joints":
                    txt = ", ".join("{:.1f}".format(a) for a in val)
                elif val is not None:
                    T = np.asarray(val)
                    txt = "{:.1f}, {:.1f}, {:.1f} mm".format(*(T[:3, 3] * 1000))
                h.addWidget(QLabel(txt), 1)
                b = QPushButton("Teach current")
                b.clicked.connect(lambda *_, k=key: self.teach_into.emit(self.step, k))
                h.addWidget(b)
            else:
                w = QLineEdit(str(val))
                w.textChanged.connect(lambda v, k=key: self._set(k, v))
            self.form.addRow("{}{}".format(label, " ({})".format(unit) if unit else ""), w)

    def _tool_row(self, spec, key, label):
        w = QComboBox()
        w.addItem("(keep current)", "")
        fam = spec.tool_family
        for t in self.tools:
            if fam in ("", "any") or fam in lc.tool_operations(t.kind):
                w.addItem("{}  {}".format(t.id, t.name), t.id)
        cur = self.step.params.get(key, "")
        i = w.findData(cur)
        w.setCurrentIndex(max(i, 0))
        w.currentIndexChanged.connect(lambda *_: self._set(key, w.currentData()))
        self.form.addRow(label, w)

    def _set(self, k, v):
        if self.step is not None:
            self.step.params[k] = v
            self.edited.emit()

    def _set_label(self, t):
        if self.step is not None:
            self.step.label = t
            self.edited.emit()


# ---------------------------------------------------------------- background expansion
class Expander(QObject):
    done = Signal(object, object)   # expansion, error text
    progress = Signal(str)

    def __init__(self, station, robot, prog, tools, part, q0):
        super().__init__()
        self.args = (station, robot, prog, tools, part, q0)

    def run(self):
        try:
            st, robot, prog, tools, part, q0 = self.args
            ex = program.expand(st, robot, prog, tools, part, q0,
                                progress=lambda i, n, t: self.progress.emit("{}/{} {}".format(i + 1, n, t)))
            self.done.emit(ex, None)
        except Exception:   # noqa: BLE001
            self.done.emit(None, traceback.format_exc())


# ---------------------------------------------------------------- main window
class Studio(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("lithocam studio")
        self.resize(1600, 960)
        self.station = program.Station()
        self.part = None                 # (tris mm, TriMesh)
        self.part_tris = None
        self.tool_lib: List = []
        self.expansion: Optional[program.Expansion] = None
        self.sim_q: np.ndarray = np.zeros((0, 6))
        self.view = CellView()
        self._threads = []

        # centre: view + simulation bar
        centre = QWidget()
        cv = QVBoxLayout(centre)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.addWidget(QWidget.createWindowContainer(self.view.win), 1)
        bar = QHBoxLayout()
        st = self.style()
        self.btn_play = QToolButton()
        self.btn_play.setIcon(st.standardIcon(QStyle.SP_MediaPlay))
        self.btn_play.setCheckable(True)
        self.btn_play.toggled.connect(self._play)
        self.sim = QSlider(Qt.Horizontal)
        self.sim.valueChanged.connect(self._scrub)
        self.sim_speed = QComboBox()
        for s in (1, 2, 5, 10, 25, 100):
            self.sim_speed.addItem("x{}".format(s), s)
        self.sim_speed.setCurrentIndex(2)
        self.sim_label = QLabel("no simulation")
        bar.addWidget(self.btn_play)
        bar.addWidget(self.sim, 1)
        bar.addWidget(self.sim_speed)
        bar.addWidget(self.sim_label)
        cv.addLayout(bar)
        self.setCentralWidget(centre)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)

        # docks
        self.tree = WorkTree()
        self.tree.itemSelectionChanged.connect(self._selected)
        self.tree.changed.connect(self._sync_programs)
        self.tree.request_add.connect(self._add_step_at)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._tree_menu)
        self.palette = QToolBox()
        for cat, _ in catalog.CATEGORIES:
            lst = PaletteList(catalog.steps_in(cat))
            lst.itemDoubleClicked.connect(lambda it: self._add_step(it.data(ROLE)))
            self.palette.addItem(lst, cat)
        self.tools_panel = ToolPanel()
        self.tools_panel.assign.connect(self._assign_tool)
        self.tools_panel.btn_load.clicked.connect(self._load_tools)
        self.tools_panel.btn_save.clicked.connect(self._save_tools)
        objects = QTabWidget()
        objects.addTab(self.palette, "Operations")
        objects.addTab(self.tools_panel, "Tools")
        self.robot_panel = RobotPanel()
        self.robot_panel.moved.connect(self._robot_moved)
        self.robot_panel.teach.connect(self._teach)
        self.robot_panel.robot_changed.connect(self._set_robot)
        self.robot_panel.spindle_changed.connect(self._set_spindle)
        self.props = Properties()
        self.props.edited.connect(self._step_edited)
        self.props.teach_into.connect(self._teach_into)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setFont(QFont("Consolas", 9))
        bottom = QTabWidget()
        bottom.addTab(self.log, "Log")
        bottom.addTab(self.output, "Posted program")
        self._dock("Work tree", self.tree, Qt.LeftDockWidgetArea)
        self._dock("Objects", objects, Qt.LeftDockWidgetArea)
        self._dock("Robot panel", self.robot_panel, Qt.RightDockWidgetArea, scroll=True)
        self._dock("Properties", self.props, Qt.RightDockWidgetArea, scroll=True)
        self._dock("Output", bottom, Qt.BottomDockWidgetArea)
        self._toolbars()
        self.statusBar().showMessage("Ready")

        self._load_builtin_tools()
        self._set_robot("ur10e", first=True)
        self._demo()
        self._view("iso")

    # ---------------------------------------------------------------- ui scaffolding
    def _dock(self, title, w, area, scroll=False):
        d = QDockWidget(title, self)
        d.setObjectName(title)
        if scroll:
            sa = QScrollArea()
            sa.setWidget(w)
            sa.setWidgetResizable(True)
            sa.setMinimumWidth(360)
            w = sa
        d.setWidget(w)
        self.addDockWidget(area, d)
        return d

    def _act(self, tb, text, icon, slot, tip=""):
        a = QAction(icon, text, self)
        a.setToolTip(tip or text)
        a.triggered.connect(slot)
        tb.addAction(a)
        return a

    def _toolbars(self):
        st = self.style()
        tb = QToolBar("Station")
        tb.setObjectName("Station")
        tb.setIconSize(QSize(22, 22))
        self.addToolBar(tb)
        self._act(tb, "New", st.standardIcon(QStyle.SP_FileIcon), self._new)
        self._act(tb, "Open", st.standardIcon(QStyle.SP_DialogOpenButton), self._open)
        self._act(tb, "Save", st.standardIcon(QStyle.SP_DialogSaveButton), self._save)
        tb.addSeparator()
        self._act(tb, "Load part", glyph_icon("STL", "#8d6e63", 22), self._load_part, "Load a part (STL, mm)")
        self._act(tb, "Tools", glyph_icon("T", "#455a64", 22), self._load_tools, "Load a tool library (CSV)")
        tb.addSeparator()
        for name, g in (("iso", "3D"), ("top", "Top"), ("front", "Fr"), ("right", "Rt")):
            self._act(tb, name, glyph_icon(g, "#37474f", 22), lambda *_, n=name: self._view(n), "View " + name)
        self._act(tb, "Fit", glyph_icon("[ ]", "#37474f", 22), lambda: self._view(None), "Fit all")
        tb.addSeparator()
        self._act(tb, "Program", glyph_icon("+P", "#3d7bd9", 22), self._add_program, "Add program")
        self._act(tb, "MoveJ", glyph_icon("J", "#3d7bd9", 22), lambda: self._teach("movej"), "Teach a joint move")
        self._act(tb, "MoveL", glyph_icon("L", "#3d7bd9", 22), lambda: self._teach("movel"), "Teach a linear move")
        tb.addSeparator()
        self._act(tb, "Simulate", st.standardIcon(QStyle.SP_MediaPlay), self._simulate, "Expand and simulate")
        self._act(tb, "Stop", st.standardIcon(QStyle.SP_MediaStop), self._stop)
        self._act(tb, "Post", glyph_icon("PP", "#2e7d32", 22), self._post, "Generate the robot program")

        mt = QToolBar("Machining")
        mt.setObjectName("Machining")
        mt.setIconSize(QSize(22, 22))
        self.addToolBarBreak()
        self.addToolBar(mt)
        self._act(mt, "Setup", glyph_icon("S", "#6c6f7d", 22), lambda: self._add_step("setup"),
                  "Setup: work frame and stock from the part")
        for cat, colour in catalog.CATEGORIES:
            if cat in ("Robot", "Setup"):
                continue
            b = QToolButton()
            b.setText(cat)
            b.setIcon(glyph_icon(cat[:2], colour, 22))
            b.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
            b.setPopupMode(QToolButton.InstantPopup)
            m = QMenu(b)
            for s in catalog.steps_in(cat):
                a = m.addAction(step_icon(s), "{}{}".format(s.label, "" if s.status == "done" else
                                                            "  ({})".format(s.status)))
                a.triggered.connect(lambda *_, sid=s.id: self._add_step(sid))
            b.setMenu(m)
            mt.addWidget(b)
        mt.addSeparator()
        self._act(mt, "Generate", glyph_icon("G", "#ef6c00", 22), self._simulate, "Generate all toolpaths")
        self._act(mt, "Stock check", glyph_icon("Z", "#ad1457", 22), self._stock_check,
                  "Cut the stock in a Z-map along every 3-axis toolpath and report the leftover and gouge")
        post_box = QComboBox()
        for d in ("urscript", "krl", "rapid", "cl"):
            post_box.addItem(d, d)
        self.post_box = post_box
        mt.addWidget(QLabel("  Post "))
        mt.addWidget(post_box)

    def say(self, text):
        self.log.appendPlainText(text)
        self.statusBar().showMessage(text.splitlines()[0][:200], 8000)

    # ---------------------------------------------------------------- robot and tools
    def _set_robot(self, name, first=False):
        self.station.robot = name
        self.robot = program.load_robot(name)
        self.say("Loading {} visual meshes ...".format(program.ROBOTS[name][0]))
        QApplication.processEvents()
        self.view.set_robot(self.robot, link_meshes(self.robot))
        self.post_box.setCurrentText(program.ROBOTS[name][2])
        self.robot_panel.combo.blockSignals(True)
        self.robot_panel.combo.setCurrentIndex(self.robot_panel.combo.findData(name))
        self.robot_panel.combo.blockSignals(False)
        self.robot_panel.set_robot(self.robot, program.HOME.get(name, [0.0] * self.robot.dof))
        if name == "kr120" and first is False:
            self.station.work_xyz = [1.6, 0.0, 0.3]
        elif name == "ur10e" and first is False:
            self.station.work_xyz = [0.7, 0.0, 0.0]
        if not first:
            self.station.spindle = None   # the new cell's default spindle
        key = spindles.DEFAULT_FOR_ROBOT.get(name)
        self.robot_panel.spindle.blockSignals(True)
        self.robot_panel.spindle.setCurrentIndex(max(0, self.robot_panel.spindle.findData(key)))
        self.robot_panel.spindle.blockSignals(False)
        self._mount(self._first_tool())
        self._refresh_scene()
        self._rebuild_tree()

    def _robot_moved(self, q):
        self.view.update_robot(self.robot.link_transforms(q), self.robot.tcp(q))

    def _set_spindle(self, key):
        self.station.spindle = spindles.to_dict(spindles.PRESETS[key])
        self._mount(self._first_tool())
        self._rebuild_tree()

    def _first_tool(self):
        tools = self._tools()
        for p in self.station.programs:
            for s in p.steps:
                if s.params.get("tool") in tools:
                    return tools[s.params["tool"]]
        return None

    def _mount(self, tool):
        """Show the spindle with `tool` in its collet and make the cutter tip the robot's TCP (wire tools: the bow
        from the cell URDF, flange TCP)."""
        wire = tool is not None and tool.kind in (lc.ToolKind.DiamondWire, lc.ToolKind.MultiWireSaw)
        self.view.show_wire_tool(wire)
        if wire:
            self.robot.tcp_offset = np.eye(4)
            self.view.set_tool(None, [], np.eye(4))
        else:
            sp = self.station.spindle_model()
            so = spindles.stickout(tool) if tool is not None else 0.0
            self.robot.tcp_offset = self.station.tool_tcp(tool) if tool is not None else sp.tcp(0.0)
            self.view.set_tool(sp, spindles.cutter_parts(tool, so) if tool is not None else [],
                               self.robot.tcp_offset)
        self._robot_moved(self.robot_panel.q)
        self.robot_panel.set_q(self.robot_panel.q, emit=False)

    def _load_builtin_tools(self):
        self.tool_lib = list(lc.stone_tool_library().tools) + list(lc.wood_tool_library().tools)
        self._tools_changed()

    def _tools_changed(self):
        self.tools_panel.set_tools(self.tool_lib)
        self.props.tools = self.tool_lib
        self._rebuild_tree()

    def _tools(self) -> Dict[str, object]:
        return {t.id: t for t in self.tool_lib}

    def _load_tools(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Tool library", "", "CSV (*.csv)")
        if fn:
            try:
                self.tool_lib = list(lc.read_tool_library(fn).tools)
                self.station.tool_library = fn
                self._tools_changed()
                self.say("Loaded {} tools from {}".format(len(self.tool_lib), fn))
            except Exception as e:   # noqa: BLE001
                QMessageBox.warning(self, "Tool library", str(e))

    def _save_tools(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save tool library", "tools.csv", "CSV (*.csv)")
        if fn:
            lib = lc.ToolLibrary()
            lib.tools = self.tool_lib
            ok = lc.write_tool_library(lib, fn)
            self.say("Saved tool library to {}".format(fn) if ok else "Could not write {}".format(fn))

    def _assign_tool(self, tid):
        step = self._current_step()
        if step is None:
            self.say("Select a step first, then pick its tool")
            return
        fam = step.spec.tool_family
        tool = self._tools()[tid]
        if fam and fam != "any" and fam not in lc.tool_operations(tool.kind):
            self.say("{} does not fit {} (needs a '{}' tool)".format(tid, step.spec.label, fam))
            return
        step.params["tool"] = tid
        self._rebuild_tree(select=step)

    # ---------------------------------------------------------------- tree
    def _rebuild_tree(self, select=None):
        self.tree.blockSignals(True)
        self.tree.clear()
        st = self.station
        root = QTreeWidgetItem(["Station"])
        root.setIcon(0, glyph_icon("St", "#455a64", 20))
        self.tree.addTopLevelItem(root)
        rob = QTreeWidgetItem(root, ["{} (base)".format(program.ROBOTS[st.robot][0])])
        rob.setIcon(0, glyph_icon("R", "#3d7bd9", 20))
        used = sorted({s.params.get("tool") for p in st.programs for s in p.steps if s.params.get("tool")})
        sp = st.spindle_model()
        spi = QTreeWidgetItem(rob, ["Spindle: {} ({}, gauge {:.0f} mm{})".format(
            sp.name, sp.collet, sp.gauge(), "" if sp.source != "example" else ", example size")])
        spi.setIcon(0, glyph_icon("Sp", "#78909c", 20))
        tools = QTreeWidgetItem(spi, ["Tools ({} in library)".format(len(self.tool_lib))])
        tools.setIcon(0, glyph_icon("T", "#455a64", 20))
        for tid in used:
            t = self._tools().get(tid)
            QTreeWidgetItem(tools, ["{}  {}".format(tid, t.name if t else "(missing)")])
        work = QTreeWidgetItem(root, ["Work frame  {:.3f}, {:.3f}, {:.3f} m".format(*st.work_xyz)])
        work.setIcon(0, glyph_icon("F", "#6c6f7d", 20))
        part = QTreeWidgetItem(work, ["Part: " + (Path(st.part_path).name if st.part_path else
                                                  ("demo dome" if self.part_tris is not None else "none"))])
        part.setIcon(0, glyph_icon("P", "#8d6e63", 20))
        if st.stock:
            s = st.stock
            stock = QTreeWidgetItem(work, ["Stock {:.0f} x {:.0f} x {:.0f} mm".format(
                s["xmax"] - s["xmin"], s["ymax"] - s["ymin"], s["zmax"] - s["zmin"])])
            stock.setIcon(0, glyph_icon("B", "#f2b705", 20))
        progs = QTreeWidgetItem(root, ["Programs"])
        progs.setIcon(0, glyph_icon("Pg", "#3d7bd9", 20))
        sel_item = None
        for p in st.programs:
            pi = QTreeWidgetItem(progs, [p.name])
            pi.setData(0, ROLE + 1, "program")
            pi.setData(0, ROLE + 2, p)
            pi.setIcon(0, glyph_icon("()", "#3d7bd9", 20))
            for s in p.steps:
                si = QTreeWidgetItem(pi, [s.title()])
                si.setData(0, ROLE, s)
                si.setIcon(0, step_icon(s.spec))
                if s is select:
                    sel_item = si
            pi.setExpanded(True)
        for it in (root, rob, work, progs):
            it.setExpanded(True)
        self.tree.blockSignals(False)
        if sel_item is not None:
            self.tree.setCurrentItem(sel_item)

    def _sync_programs(self):
        """Program step order from the tree (after a drag)."""
        progs = []
        for i in range(self.tree.topLevelItemCount()):
            root = self.tree.topLevelItem(i)
            for j in range(root.childCount()):
                if root.child(j).text(0) == "Programs":
                    pr = root.child(j)
                    for k in range(pr.childCount()):
                        pi = pr.child(k)
                        p = pi.data(0, ROLE + 2)
                        p.steps = [pi.child(m).data(0, ROLE) for m in range(pi.childCount())]
                        progs.append(p)
        if progs:
            self.station.programs = progs
        self._rebuild_tree()

    def _current_program(self) -> program.Program:
        it = self.tree.currentItem()
        while it is not None:
            if it.data(0, ROLE + 1) == "program":
                return it.data(0, ROLE + 2)
            it = it.parent()
        return self.station.programs[0]

    def _current_step(self) -> Optional[program.Step]:
        it = self.tree.currentItem()
        return it.data(0, ROLE) if it is not None and isinstance(it.data(0, ROLE), program.Step) else None

    def _new_step(self, sid) -> program.Step:
        spec = catalog.BY_ID[sid]
        params = catalog.default_params(spec)
        step = program.Step(sid, params)
        cur = self._current_step()
        tool_now = None
        for p in self.station.programs:
            for s in p.steps:
                if s.params.get("tool"):
                    tool_now = s.params["tool"]
        if spec.tool_family and spec.tool_family != "any":
            fits = [t.id for t in self.tool_lib if spec.tool_family in lc.tool_operations(t.kind) and
                    (lc.has_cutter(t) or sid not in cam.GENERATORS)]
            if tool_now in fits:
                step.params["tool"] = tool_now
            elif fits:
                step.params["tool"] = fits[0]
        if sid == "movej":
            step.params["joints"] = [math.degrees(v) for v in self.robot_panel.q]
        if sid == "movel":
            step.params["pose"] = self.robot.tcp(self.robot_panel.q).tolist()
        del cur
        return step

    def _add_step(self, sid):
        prog = self._current_program()
        step = self._new_step(sid)
        cur = self._current_step()
        if sid == "setup":
            self._do_setup(step)
        if cur is not None and cur in prog.steps:
            prog.steps.insert(prog.steps.index(cur) + 1, step)
        else:
            prog.steps.append(step)
        self._rebuild_tree(select=step)
        self.say("Added {}".format(step.title()))

    def _add_step_at(self, sid, target_item):
        if target_item.data(0, ROLE + 1) == "program":
            prog = target_item.data(0, ROLE + 2)
            idx = len(prog.steps)
        else:
            prog = target_item.parent().data(0, ROLE + 2)
            idx = prog.steps.index(target_item.data(0, ROLE)) + 1
        step = self._new_step(sid)
        if sid == "setup":
            self._do_setup(step)
        prog.steps.insert(idx, step)
        self._rebuild_tree(select=step)

    def _tree_menu(self, pos):
        it = self.tree.itemAt(pos)
        m = QMenu(self)
        step = it.data(0, ROLE) if it is not None else None
        if isinstance(step, program.Step):
            m.addAction("Delete step", lambda: self._delete_step(step))
            m.addAction("Duplicate step", lambda: self._dup_step(step))
        if it is not None and it.data(0, ROLE + 1) == "program":
            p = it.data(0, ROLE + 2)
            m.addAction("Simulate this program", lambda: (self.tree.setCurrentItem(it), self._simulate()))
            m.addAction("Post this program", lambda: (self.tree.setCurrentItem(it), self._post()))
            m.addAction("Delete program", lambda: self._delete_program(p))
        m.addAction("Add program", self._add_program)
        m.exec(self.tree.viewport().mapToGlobal(pos))

    def _delete_step(self, step):
        for p in self.station.programs:
            if step in p.steps:
                p.steps.remove(step)
        self._rebuild_tree()

    def _dup_step(self, step):
        for p in self.station.programs:
            if step in p.steps:
                s2 = program.Step(step.kind, dict(step.params), step.label)
                p.steps.insert(p.steps.index(step) + 1, s2)
                self._rebuild_tree(select=s2)
                return

    def _add_program(self):
        self.station.programs.append(program.Program("Prog{}".format(len(self.station.programs) + 1)))
        self._rebuild_tree()

    def _delete_program(self, p):
        if len(self.station.programs) > 1:
            self.station.programs.remove(p)
            self._rebuild_tree()

    def _selected(self):
        self.props.show_step(self._current_step())

    def _step_edited(self):
        it = self.tree.currentItem()
        step = self._current_step()
        if it is not None and step is not None:
            it.setText(0, step.title())

    # ---------------------------------------------------------------- teach
    def _teach(self, kind):
        self._add_step(kind)

    def _teach_into(self, step, key):
        if key == "joints":
            step.params["joints"] = [math.degrees(v) for v in self.robot_panel.q]
        else:
            step.params["pose"] = self.robot.tcp(self.robot_panel.q).tolist()
        self._rebuild_tree(select=step)
        self.props.show_step(step)

    # ---------------------------------------------------------------- part, setup, scene
    def _demo(self):
        from .cam import trimesh
        n, size, h = 30, 120.0, 25.0
        xs = np.linspace(-size / 2, size / 2, n + 1)
        Z = lambda x, y: h * max(0.0, 1 - (x * x + y * y) / (size / 2) ** 2) + 5.0   # noqa: E731
        tris = []
        for i in range(n):
            for j in range(n):
                a, b = (xs[i], xs[j], Z(xs[i], xs[j])), (xs[i + 1], xs[j], Z(xs[i + 1], xs[j]))
                c, d = (xs[i + 1], xs[j + 1], Z(xs[i + 1], xs[j + 1])), (xs[i], xs[j + 1], Z(xs[i], xs[j + 1]))
                tris += [(a, b, c), (a, c, d)]
        self.part_tris = np.array(tris)
        self.part = (self.part_tris, trimesh(self.part_tris))
        self.station.stock = cam.stock_box(self.part_tris, 5.0)
        self.station.stock["zmin"] = 0.0
        self.station.programs = [program.Program("Main", [
            program.Step("set_tool", {"tool": "W7"}),
            program.Step("movej", {"joints": [math.degrees(v) for v in program.HOME[self.station.robot]]}),
            program.Step("rough_plane", {"tool": "W2", "stepdown": 8.0, "stepover": 4.0, "allowance": 1.0}),
            program.Step("raster", {"tool": "W7", "stepover": 2.0}),
            program.Step("polish", {"tool": "S8"}),
            program.Step("movej", {"joints": [math.degrees(v) for v in program.HOME[self.station.robot]]}),
        ])]
        self._mount(self._first_tool())
        self._refresh_scene()
        self._rebuild_tree()
        self.say("Demo station: a dome on a {:.0f} mm stock; the program roughs, finishes and lists a polish step. "
                 "Press Simulate.".format(size + 10))

    def _load_part(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Part", "", "STL (*.stl)")
        if not fn:
            return
        try:
            tris = cam.load_stl(fn)
            self.part_tris = tris
            self.part = (tris, cam.trimesh(tris))
            self.station.part_path = fn
            self.station.stock = cam.stock_box(tris, 5.0)
            self._refresh_scene()
            self._rebuild_tree()
            lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
            self.say("Part {}: {} triangles, {:.0f} x {:.0f} x {:.0f} mm".format(Path(fn).name, len(tris),
                                                                                 *(hi - lo)))
        except Exception as e:   # noqa: BLE001
            QMessageBox.warning(self, "Part", str(e))

    def _do_setup(self, step):
        if self.part_tris is None:
            self.say("Setup needs a part")
            return
        self.station.stock = cam.stock_box(self.part_tris, float(step.params.get("stock_margin", 5.0)))
        self._refresh_scene()

    def _refresh_scene(self):
        W = self.station.work_frame()
        self.view.set_part(self.part_tris, W)
        self.view.set_stock(self.station.stock, W)
        self.robot_panel.work = W
        self._robot_moved(self.robot_panel.q)

    def _view(self, which):
        tf = self.robot.link_transforms(self.robot_panel.q)
        pts = [T[:3, 3] for T in tf.values()] + [self.station.work_frame()[:3, 3]]
        lo, hi = np.min(pts, 0) - 0.15, np.max(pts, 0) + 0.15
        self.view.win.view(which or "keep", (lo, hi))
        self.view.win.center = (self.view.win.center + self.station.work_frame()[:3, 3]) / 2
        self.view.win.dist *= 0.95
        self.view.win.update_camera()

    # ---------------------------------------------------------------- simulate and post
    def _simulate(self):
        prog = self._current_program()
        self.say("Expanding '{}' on the {} ...".format(prog.name, program.ROBOTS[self.station.robot][0]))
        th = QThread(self)
        wk = Expander(self.station, program.load_robot(self.station.robot), prog, self._tools(), self.part,
                      self.robot_panel.q)
        wk.moveToThread(th)
        th.started.connect(wk.run)
        wk.progress.connect(self._progress)   # a bound slot runs queued in the GUI thread; a lambda would not
        wk.done.connect(self._expanded)
        wk.done.connect(th.quit)
        self._threads.append((th, wk))
        th.start()

    def _progress(self, text):
        self.statusBar().showMessage(text)

    def _expanded(self, ex, err):
        if err:
            self.say("Expansion failed:\n" + err)
            return
        self.expansion = ex
        for m in ex.messages:
            self.say("  " + m)
        q = ex.joints()
        self.sim_q = q
        self.sim_seg_end = np.cumsum([len(s.q) for s in ex.segments])
        self._sim_seg = -1
        if len(q) == 0:
            self.say("Nothing to simulate")
            return
        pts = np.vstack([s.tcp for s in ex.segments])
        feed = np.concatenate([s.feed for s in ex.segments])
        ok = np.concatenate([s.ok for s in ex.segments])
        self.view.set_path(pts, feed, ok)
        self.sim.blockSignals(True)
        self.sim.setRange(0, len(q) - 1)
        self.sim.setValue(0)
        self.sim.blockSignals(False)
        self._scrub(0)
        self.say("Simulation ready: {} robot samples, {} unreachable. Press play or drag the bar.".format(
            len(q), ex.unreachable))
        self._post(show_only=True)
        if getattr(self, "smoke", None):
            QTimer.singleShot(300, self._smoke_capture)
        else:
            self.btn_play.setChecked(True)

    def _smoke_capture(self):
        """--smoke: scrub to the given fraction, grab the window from the screen, write the log, quit."""
        out, frac = self.smoke
        self.sim.setValue(int(self.sim.maximum() * frac))
        QApplication.processEvents()

        def grab():
            g = self.frameGeometry()
            pm = QApplication.primaryScreen().grabWindow(0, g.x(), g.y(), g.width(), g.height())
            pm.save(out)
            Path(out).with_suffix(".log.txt").write_text(self.log.toPlainText())
            QApplication.quit()
        QTimer.singleShot(1500, grab)

    def _play(self, on):
        self.btn_play.setIcon(self.style().standardIcon(QStyle.SP_MediaPause if on else QStyle.SP_MediaPlay))
        if on and len(self.sim_q):
            if self.sim.value() >= self.sim.maximum():
                self.sim.setValue(0)
            self.timer.start(30)
        else:
            self.timer.stop()

    def _tick(self):
        v = self.sim.value() + self.sim_speed.currentData()
        if v >= self.sim.maximum():
            v = self.sim.maximum()
            self.btn_play.setChecked(False)
        self.sim.setValue(v)

    def _scrub(self, i):
        if not len(self.sim_q):
            return
        i = int(min(max(i, 0), len(self.sim_q) - 1))
        q = self.sim_q[i]
        robot = self.robot
        k = int(np.searchsorted(self.sim_seg_end, i, side="right"))
        if k != self._sim_seg and k < len(self.expansion.segments):   # tool of this segment: TCP and cylinder
            self._sim_seg = k
            seg = self.expansion.segments[k]
            robot.tcp_offset = seg.tcp_offset
            self._mount(self._tools().get(seg.tool))
            robot.tcp_offset = seg.tcp_offset
            self.sim_label.setToolTip(seg.label)
        self.robot_panel.set_q(q, emit=False)
        self.view.update_robot(robot.link_transforms(q), robot.tcp(q))
        self.sim_label.setText("{}/{}".format(i + 1, len(self.sim_q)))

    def _stop(self):
        self.btn_play.setChecked(False)
        self.sim.setValue(0)

    def _post(self, show_only=False):
        if self.expansion is None:
            self.say("Simulate first: the post uses the verified joint solutions")
            return
        dialect = self.post_box.currentData()
        text = post.write(self.expansion.records, dialect)
        self.output.setPlainText(text)
        back = post.joint_targets(text, dialect)
        rec_q = [r.data["q"] for r in self.expansion.records if r.kind in ("movej", "movel")]
        same = len(back) == len(rec_q) and (not back or np.max(np.abs(np.asarray(back) - np.asarray(rec_q))) < 1e-5)
        self.say("Posted {} lines ({}); read-back check: {}".format(text.count("\n"), dialect,
                                                                      "PASS" if same else "FAIL"))
        if show_only:
            return
        fn, _ = QFileDialog.getSaveFileName(self, "Save robot program",
                                            self._current_program().name + post.EXTENSIONS[dialect])
        if fn:
            Path(fn).write_text(text)
            self.say("Saved {}".format(fn))

    def _stock_check(self):
        if self.part is None or self.station.stock is None:
            self.say("Stock check needs a part and a stock")
            return
        tools = self._tools()
        tris, mesh = self.part
        for p in self.station.programs:
            for s in p.steps:
                if s.kind in cam.GENERATORS and s.params.get("tool") in tools:
                    tool = tools[s.params["tool"]]
                    if not lc.has_cutter(tool):
                        continue
                    params = dict(catalog.default_params(s.spec), **s.params)
                    params.pop("tool", None)
                    moves = cam.GENERATORS[s.kind](tris, mesh, tool, self.station.stock, **params)
                    stats, _ = cam.zmap_check(tris, mesh, tool, moves, self.station.stock, 1.0)
                    self.say("Z-map {} / {}: leftover max {:.3f} mm, gouge {:.3f} mm over {} cells".format(
                        p.name, s.title(), stats.max_dev, -min(0.0, stats.min_dev), stats.cells))

    # ---------------------------------------------------------------- files
    def _new(self):
        self.station = program.Station(robot=self.station.robot)
        self.part = self.part_tris = None
        self.expansion = None
        self.view.set_path(None, None)
        self._refresh_scene()
        self._rebuild_tree()

    def _open(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Open station", "", "lithocam station (*.lcs.json *.json)")
        if not fn:
            return
        try:
            st = program.Station.from_json(Path(fn).read_text())
            self.station = st
            if st.tool_library:
                self.tool_lib = list(lc.read_tool_library(st.tool_library).tools)
                self._tools_changed()
            if st.part_path and Path(st.part_path).exists():
                tris = cam.load_stl(st.part_path)
                self.part_tris, self.part = tris, (tris, cam.trimesh(tris))
            self._set_robot(st.robot, first=True)
            self.say("Opened {}".format(fn))
        except Exception as e:   # noqa: BLE001
            QMessageBox.warning(self, "Open", str(e))

    def _save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save station", "station.lcs.json", "lithocam station (*.json)")
        if fn:
            Path(fn).write_text(self.station.to_json())
            self.say("Saved {}".format(fn))


def main(argv=None) -> int:
    """Options: --robot ur10e|kr120; --smoke OUT.png [FRACTION]: simulate the demo, capture, exit (CI and docs)."""
    import faulthandler
    if sys.stderr is not None:
        faulthandler.enable()
    argv = list(argv if argv is not None else sys.argv)
    app = QApplication(argv)
    app.setApplicationName("lithocam studio")
    app.setStyle("Fusion")
    w = Studio()
    if "--robot" in argv:
        w._set_robot(argv[argv.index("--robot") + 1])
        w._demo()
    w.show()
    w.showMaximized()
    if "--smoke" in argv:
        k = argv.index("--smoke")
        frac = float(argv[k + 2]) if len(argv) > k + 2 and not argv[k + 2].startswith("--") else 0.6
        w.smoke = (argv[k + 1], frac)
        QTimer.singleShot(1500, w._simulate)
    return app.exec()
