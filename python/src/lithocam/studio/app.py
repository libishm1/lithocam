# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""lithocam studio: one window for stone CAM and robot programming.

Layout (work tree and objects browser, machining operation toolbars, 3D view, robot panel, output tabs):

  toolbars   Station (open, save, part, tools, views, teach, simulate, post) and Machining (setup and one
             drop-down per operation category, generate, stock check, cut stock, process check)
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
from PySide6.QtGui import QAction, QBrush, QColor, QFont, QIcon, QKeySequence, QPainter, QPixmap
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox,
                               QDockWidget, QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
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
    save_target = Signal()
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
        for text, slot in (("Home", self._home), ("Save target", self.save_target.emit),
                           ("Teach MoveJ", lambda: self.teach.emit("movej")),
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
    table_mode_changed = Signal(str)   # the station's table driven mode: operator, e1, io

    def __init__(self):
        super().__init__()
        self.form = QFormLayout(self)
        self.step: Optional[program.Step] = None
        self.tools: List = []
        self.targets: List = []          # the station's named targets (list of dicts, shared)

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
            elif kind == "target":
                w = QComboBox()
                w.addItem("(none: use the stored values)", "")
                for t in self.targets:
                    w.addItem(t["name"], t["name"])
                w.setCurrentIndex(max(w.findData(val or ""), 0))
                w.currentIndexChanged.connect(lambda *_, k=key, ww=w: self._set(k, ww.currentData()))
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

    def show_station(self, st, on_change, app=None):
        """Station settings that change the posted program: work frame, material, robot factor, tool length,
        rapid speed, turntable."""
        while self.form.rowCount():
            self.form.removeRow(0)
        self.step = None
        self.form.addRow(QLabel("<b>Station</b> <span style='color:#888'>(settings that change the posted "
                                "program)</span>"))

        def spin(val, lo, hi, dec, cb, suffix=""):
            w = QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setDecimals(dec)
            w.setValue(float(val))
            if suffix:
                w.setSuffix(suffix)
            w.valueChanged.connect(lambda v: (cb(v), on_change()))
            return w
        for k, axis in enumerate("xyz"):
            self.form.addRow("Work frame {} (m)".format(axis),
                             spin(st.work_xyz[k], -10, 10, 4, lambda v, k=k: st.work_xyz.__setitem__(k, v)))
        for k, axis in enumerate(("roll", "pitch", "yaw")):
            self.form.addRow("Work frame {} (deg)".format(axis),
                             spin(st.work_rpy[k], -360, 360, 2, lambda v, k=k: st.work_rpy.__setitem__(k, v)))
        from .. import cutting_data, materials
        mats = materials.available()
        have = {e.material for e in cutting_data.TABLE}
        mc = QComboBox()
        for n in mats:
            mc.addItem(n if n in have else n + "   (no cutting data)", n)
        if st.material not in mats:
            mc.addItem(st.material, st.material)
        mc.setCurrentIndex(max(0, mc.findData(st.material)))

        def mat_tip(name):
            try:
                return ("Cutting data (feeds, speeds, wire sawing rate) are looked up for this material.\n"
                        "Which quantities are sourced (lithocam.materials.report):\n" + materials.report([name]))
            except KeyError as e:
                return "No sourced row for '{}': {}".format(name, e)

        def pick_material(_):
            st.material = mc.currentData()
            mc.setToolTip(mat_tip(st.material))
            on_change()
        mc.setToolTip(mat_tip(st.material))
        mc.currentIndexChanged.connect(pick_material)
        self.form.addRow("Material", mc)
        self.form.addRow("Robot feed factor", spin(st.robot_factor, 0.01, 2.0, 2,
                                                   lambda v: setattr(st, "robot_factor", v)))
        self.form.addRow("Tool length override (mm, 0 = spindle + tool)",
                         spin(st.tool_length, 0, 2000, 1, lambda v: setattr(st, "tool_length", v)))
        self.form.addRow("Rapid speed near the stone (mm/s)",
                         spin(st.rapid_speed, 1, 2000, 1, lambda v: setattr(st, "rapid_speed", v)))
        if st.turntable:
            self.form.addRow("Turntable diameter (m)", spin(st.turntable.get("diameter", 0.8), 0.05, 5, 3,
                                                            lambda v: st.turntable.__setitem__("diameter", v)))
            self._table_fields(st, on_change, spin)
        # cell I/O: how the controller switches the spindle, water, wire motor and tool change
        io = QComboBox()
        io.addItem("None (posted as comments, with a warning)", "")
        io.addItem("Cell default (cells/<robot>_io.json, placeholders)", "default")
        if getattr(st, "cell_io", "") not in ("", "default"):
            io.addItem(Path(st.cell_io).name, st.cell_io)
        io.addItem("Choose a file ...", "?")
        io.setCurrentIndex(max(0, io.findData(getattr(st, "cell_io", ""))))

        def pick_io(_):
            v = io.currentData()
            if v == "?":
                fn, _f = QFileDialog.getOpenFileName(self, "Cell I/O definition", "", "JSON (*.json)")
                if not fn:
                    io.setCurrentIndex(0)
                    return
                v = fn
            st.cell_io = v
            on_change()
        io.currentIndexChanged.connect(pick_io)
        io.setToolTip("Spindle, water, wire motor and tool change outputs of your cell (docs/CELL_IO.md)")
        self.form.addRow("Cell I/O", io)
        if app is not None:
            gm = QComboBox()
            gm.addItem("Generic 3-axis (placeholder)", "")
            mdir = _examples_dir().parent / "gcode_machines"
            for f in sorted(mdir.glob("*.json")):
                gm.addItem(f.stem.replace("_", " "), str(f))
            gm.addItem("Choose a file ...", "?")
            gm.setCurrentIndex(max(0, gm.findData(app.gcode_machine)))

            def pick_gm(_):
                v = gm.currentData()
                if v == "?":
                    fn, _f = QFileDialog.getOpenFileName(self, "Milling machine definition", "", "JSON (*.json)")
                    v = fn or ""
                app.gcode_machine = v
                on_change()
            gm.currentIndexChanged.connect(pick_gm)
            gm.setToolTip("Machine definition used when posting ISO G-code (docs/POSTPROCESSORS.md)")
            self.form.addRow("G-code machine", gm)

    def _table_fields(self, st, on_change, spin):
        """Who turns the table (operator, E1 or PLC I/O) and the E1 axis values. E1 is a coordinated external axis of
        a KUKA or ABB controller: it is disabled for a cell that cannot post it, with the reason."""
        ts = program.table_settings(st.turntable)
        dc = QComboBox()
        for key, label in (("operator", "Operator (the program stops)"), ("e1", "E1 (coordinated external axis)"),
                           ("io", "PLC through I/O (cell I/O request and done)")):
            dc.addItem(label, key)
        probe = program.Station(robot=st.robot, turntable={"driven": "e1"})
        why = program.check_table(probe)[1]
        e1_ok = not why
        it = dc.model().item(1)
        if not e1_ok:
            it.setEnabled(False)
            it.setToolTip(why[0])
        dc.setCurrentIndex(max(0, dc.findData(ts["driven"])))
        dc.setToolTip("Operator: the program stops while the operator turns the table.\n"
                      "E1: the table is an external axis of the controller, moved together with the arm.\n"
                      "PLC I/O: a request output and a done input of the cell I/O.\n"
                      + ("" if e1_ok else "E1 is not available here: " + why[0]))
        self.form.addRow("Table driven by", dc)
        status = QLabel()
        status.setWordWrap(True)
        e1_rows = []

        def show_status():
            mode, errs, warns = program.check_table(st)
            lines = ["<span style='color:#c0392b'>{}</span>".format(e) for e in errs] + \
                    ["<span style='color:#b8860b'>{}</span>".format(w) for w in warns]
            status.setText("<br>".join(lines))
            for w in e1_rows:
                w.setEnabled(ts_mode() == "e1")

        def ts_mode():
            return str(st.turntable.get("driven", "operator") or "operator")

        def pick(_):
            st.turntable["driven"] = dc.currentData()
            show_status()
            on_change()
            self.table_mode_changed.emit(dc.currentData())
        dc.currentIndexChanged.connect(pick)
        ph = set(ts["placeholders"])

        def field(label, key, val, lo, hi, dec, to_store):
            mark = "  [placeholder]" if key in ph else ""
            w = spin(val, lo, hi, dec, to_store)
            w.setToolTip("PLACEHOLDER: no value comes from a real table; set it for your cell." if key in ph else
                         "Set for your cell")
            self.form.addRow(label + mark, w)
            e1_rows.append(w)
        lo_, hi_ = ts["e1_limits_deg"]

        def set_limit(k, v):
            cur = list(program.table_settings(st.turntable)["e1_limits_deg"])
            cur[k] = v
            st.turntable["e1_limits_deg"] = cur
        field("E1 travel, low (deg)", "e1_limits_deg", lo_, -3600, 3600, 1, lambda v: set_limit(0, v))
        field("E1 travel, high (deg)", "e1_limits_deg", hi_, -3600, 3600, 1, lambda v: set_limit(1, v))
        field("E1 speed (deg/s)", "e1_speed_deg_s", ts["e1_speed_deg_s"], 0.1, 3600, 1,
              lambda v: st.turntable.__setitem__("e1_speed_deg_s", v))
        field("E1 largest step per ruling (deg)", "e1_max_step_deg", ts["e1_max_step_deg"], 0.1, 360, 1,
              lambda v: st.turntable.__setitem__("e1_max_step_deg", v))
        self.form.addRow(status)
        show_status()

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

    def __init__(self, station, robot, prog, tools, part, q0, collisions=False, leftover=None):
        super().__init__()
        self.args = (station, robot, prog, tools, part, q0)
        self.collisions = collisions
        self.leftover = leftover      # livestock.LiveStock of the last stock run: the stock before each rest step

    def run(self):
        try:
            st, robot, prog, tools, part, q0 = self.args
            backend = None
            if self.collisions:   # Tesseract objects are created and used in this worker thread only
                from .planner import make_backend
                self.progress.emit("Loading the Tesseract cell (collision meshes) ...")
                backend = make_backend(st, program.CELLS)
            ex = program.expand(st, robot, prog, tools, part, q0, backend=backend, leftover=self.leftover,
                                progress=lambda i, n, t: self.progress.emit("{}/{} {}".format(i + 1, n, t)))
            self.done.emit(ex, None)
        except Exception:   # noqa: BLE001
            self.done.emit(None, traceback.format_exc())


class StockWorker(QObject):
    """studio.process.simulate_stock in a worker thread. `gen` tells a stale run from the current one."""
    done = Signal(int, object, object)   # generation, StockRun, error text
    progress = Signal(str)

    def __init__(self, gen, station, robot, expansion, tools):
        super().__init__()
        self.gen = gen
        self.args = (station, robot, expansion, tools)

    def run(self):
        try:
            from . import process
            st, robot, ex, tools = self.args
            run = process.simulate_stock(st, robot, ex, tools,
                                         progress=lambda i, n, t: self.progress.emit(t))
            self.done.emit(self.gen, run, None)
        except Exception:   # noqa: BLE001
            self.done.emit(self.gen, None, traceback.format_exc())


class ProcessWorker(QObject):
    """studio.process.process_check in a worker thread."""
    done = Signal(int, object, object)   # generation, ProcessReport, error text
    progress = Signal(str)

    def __init__(self, gen, station, robot, run, expansion, tools, constants, prog=None):
        super().__init__()
        self.gen = gen
        self.args = (station, run, expansion, robot, tools, constants, prog)

    def run(self):
        try:
            from . import process
            st, run, ex, robot, tools, c, prog = self.args
            rep = process.process_check(st, run, ex, robot, tools, c, progress=self.progress.emit, prog=prog)
            self.done.emit(self.gen, rep, None)
        except Exception:   # noqa: BLE001
            self.done.emit(self.gen, None, traceback.format_exc())


class ConstantsDialog(QDialog):
    """Process-check constants: one field per constant, prefilled from the sourced rows (each labelled with its
    source). A blank field skips the checks that need it."""

    def __init__(self, parent, material, robot, previous=None):
        super().__init__(parent)
        from . import process
        self.setWindowTitle("Process check constants")
        self.setMinimumWidth(720)
        v = QVBoxLayout(self)
        head = QLabel("Material <b>{}</b>, robot <b>{}</b>. Values from lithocam.materials_mechanics are prefilled "
                      "with their source; edit or clear any field. A blank field skips the check that needs it."
                      .format(material, robot))
        head.setWordWrap(True)
        v.addWidget(head)
        form = QFormLayout()
        self.sourced = process.sourced_constants(material, robot)
        self.fields = {}
        previous = previous or {}
        for c in process.CONSTANTS + process.OPTIONAL:
            box = QWidget()
            bl = QVBoxLayout(box)
            bl.setContentsMargins(0, 0, 0, 4)
            ed = QLineEdit()
            s = self.sourced.get(c.key)
            if c.key in previous:
                ed.setText("" if previous[c.key] is None else "{:g}".format(previous[c.key]))
            elif s is not None:
                ed.setText("{:g}".format(s.value))
            src = QLabel(("source: " + s.text) if s is not None else
                         "no sourced row: enter a value, or leave blank to skip the {}".format(c.used_for))
            src.setWordWrap(True)
            src.setTextInteractionFlags(Qt.TextSelectableByMouse)
            src.setStyleSheet("color: #888; font-size: 8pt")
            bl.addWidget(ed)
            bl.addWidget(src)
            form.addRow("{} ({})\n{}".format(c.key, c.unit, c.used_for), box)
            self.fields[c.key] = ed
        w = QWidget()
        w.setLayout(form)
        sa = QScrollArea()
        sa.setWidget(w)
        sa.setWidgetResizable(True)
        sa.setMinimumHeight(420)
        v.addWidget(sa, 1)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)

    def values(self):
        """{key: value} for fields the user set or changed, {key: None} for cleared sourced fields; unchanged
        sourced fields are left out, so the report names their row."""
        out = {}
        for k, ed in self.fields.items():
            t = ed.text().strip()
            s = self.sourced.get(k)
            if not t:
                if s is not None:
                    out[k] = None
                continue
            try:
                val = float(t)
            except ValueError:
                raise ValueError("{}: '{}' is not a number".format(k, t))
            if s is None or abs(val - s.value) > 1e-12 * max(abs(s.value), 1.0):
                out[k] = val
        return out


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
        self.stock_run = None            # studio.process.StockRun of the current expansion
        self.process_report = None       # studio.process.ProcessReport of stock_run
        self._stock_gen = 0              # bumps on every new stock run; stale worker results are dropped
        self._stock_busy = False
        self._stock_shown = None         # frame position shown
        self._after_stock = None         # callable run once the stock run is in (batch modes, process check)
        self._after_process = None
        self._constants_given = {}       # the dialog's last entries
        self.step_feeds = {}             # id(Step) -> process.StepFeed of the current stock run
        self._yaw = 0.0                  # table angle the 3D view shows (radians)
        self.gcode_machine = ""          # milling machine definition for the G-code post ("" = generic 3-axis)

        # centre: view + simulation bar
        centre = QWidget()
        cv = QVBoxLayout(centre)
        cv.setContentsMargins(0, 0, 0, 0)
        self.view_container = QWidget.createWindowContainer(self.view.win)
        self.view_container.setMinimumSize(480, 420)
        cv.addWidget(self.view_container, 1)
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
        self.tree.itemDoubleClicked.connect(
            lambda it, _c: self._goto_target(it.data(0, ROLE + 2)) if it.data(0, ROLE + 1) == "target" else None)
        self.palette = QToolBox()          # filled by _fill_operations
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
        self.robot_panel.save_target.connect(lambda: self._save_target())
        self.robot_panel.frame.currentIndexChanged.connect(lambda *_: self._robot_moved(self.robot_panel.q))
        self.view.win.on_gizmo = self._gizmo_moved
        self.props = Properties()
        self.props.edited.connect(self._step_edited)
        self.props.teach_into.connect(self._teach_into)
        self.props.table_mode_changed.connect(self._table_mode_changed)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setFont(QFont("Consolas", 9))
        from .trajplot import TrajectoryPlot
        self.traj = TrajectoryPlot()
        self.traj.seek.connect(lambda i: self.sim.setValue(i))
        bottom = QTabWidget()
        self.issues = QListWidget()
        self.issues.setToolTip("Errors block posting; warnings need a look. Generate refreshes the list.")
        bottom.addTab(self.issues, "Issues")
        bottom.addTab(self.traj, "Joint trajectory")
        bottom.addTab(self.log, "Log")
        bottom.addTab(self.output, "Posted program")
        self.process_text = QPlainTextEdit()
        self.process_text.setReadOnly(True)
        self.process_text.setFont(QFont("Consolas", 9))
        self.process_text.setPlainText("Process check: simulate, then press Process check on the Machining toolbar.")
        bottom.addTab(self.process_text, "Process")
        self.bottom_tabs = bottom
        self.checklist = QListWidget()
        self.checklist.setToolTip("Part to program, in order. Grey: to do; green: done; amber: needs a look.")
        self.checklist.itemDoubleClicked.connect(self._checklist_go)
        d_check = self._dock("Part to program", self.checklist, Qt.LeftDockWidgetArea)
        d_tree = self._dock("Work tree", self.tree, Qt.LeftDockWidgetArea)
        d_obj = self._dock("Objects", objects, Qt.LeftDockWidgetArea)
        d_rob = self._dock("Robot panel", self.robot_panel, Qt.RightDockWidgetArea, scroll=True)
        d_rob.widget().setMinimumHeight(380)
        d_props = self._dock("Properties", self.props, Qt.RightDockWidgetArea, scroll=True)
        d_out = self._dock("Output", bottom, Qt.BottomDockWidgetArea)
        # the side columns run the full height; the output tabs sit only under the 3D view
        self.setCorner(Qt.BottomLeftCorner, Qt.LeftDockWidgetArea)
        self.setCorner(Qt.BottomRightCorner, Qt.RightDockWidgetArea)
        self._dock_sizes = ([d_check, d_tree, d_obj], [200, 380, 320], [d_rob, d_props], [430, 520], d_out)
        self._posted_key = None
        self._toolbars()
        self.statusBar().showMessage("Ready")

        self._load_builtin_tools()
        self._set_robot("ur10e", first=True)
        self._demo()
        self._view("iso")
        self.tree.setCurrentItem(self.tree.topLevelItem(0))     # Station settings in Properties at start

    def showEvent(self, e):
        super().showEvent(e)
        if getattr(self, "_dock_sizes", None):             # Qt applies dock sizes only once the window is shown
            left, lsz, right, rsz, out = self._dock_sizes
            self._dock_sizes = None
            def apply():
                h = max(self.height(), 600)
                self.resizeDocks([out], [int(h * 0.2)], Qt.Vertical)        # the 3D view keeps most of the height
                self.resizeDocks(left, [int(v * h / 1000) for v in lsz], Qt.Vertical)
                self.resizeDocks(right, [int(v * h / 1000) for v in rsz], Qt.Vertical)
            QTimer.singleShot(400, apply)

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
        ex = QToolButton()
        ex.setText("Examples")
        ex.setToolTip("Open a ready-made station")
        ex.setToolButtonStyle(Qt.ToolButtonTextOnly)
        ex.setPopupMode(QToolButton.InstantPopup)
        em = QMenu(ex)
        em.addAction("Dome: raster and 5-axis finishing (built in)", self._demo_five_axis)
        for f in sorted(_examples_dir().glob("*.lcs.json")):
            title = f.name[:-len(".lcs.json")].replace("_", " ")
            try:
                import json as _json
                title = _json.loads(f.read_text(encoding="utf-8")).get("title") or title
            except Exception:   # noqa: BLE001
                pass
            em.addAction(title, lambda *_, p=str(f): self.open_station(p))
        ex.setMenu(em)
        tb.addWidget(ex)
        self._act(tb, "Save", st.standardIcon(QStyle.SP_DialogSaveButton), self._save)
        tb.addSeparator()
        self._act(tb, "Part", glyph_icon("STL", "#8d6e63", 22), self._load_part, "Load a part (STL, mm)")
        vb = QToolButton()
        vb.setText("View")
        vb.setIcon(glyph_icon("3D", "#37474f", 22))
        vb.setPopupMode(QToolButton.InstantPopup)
        vm = QMenu(vb)
        for name, label in (("iso", "3D"), ("top", "Top"), ("front", "Front"), ("right", "Right")):
            vm.addAction(label, lambda *_, n=name: self._view(n))
        vm.addAction("Fit all", lambda: self._view(None))
        vb.setMenu(vm)
        vb.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        ex.setIcon(glyph_icon("Ex", "#5c6bc0", 22))
        ex.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        tb.addWidget(vb)
        tb.addSeparator()
        self.act_gizmo = self._act(tb, "Move TCP", glyph_icon("✥", "#00897b", 22), self._toggle_gizmo,
                                   "Drag the tool with the mouse (T); Ctrl+T saves the position as a target")
        self.act_gizmo.setCheckable(True)
        self.act_gizmo.setShortcut(QKeySequence("T"))
        tb.addSeparator()
        a = self._act(tb, "Generate", glyph_icon("G", "#ef6c00", 22), self._simulate,
                      "Generate the program's toolpaths and robot motion (F5)")
        a.setShortcut(QKeySequence("F5"))
        self._act(tb, "Post", glyph_icon("PP", "#2e7d32", 22), self._post, "Write the robot program (Ctrl+P)"
                  ).setShortcut(QKeySequence("Ctrl+P"))
        tb.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        # teaching shortcuts without toolbar buttons (the robot panel has the buttons)
        for key, slot in (("J", lambda: self._teach("movej")), ("L", lambda: self._teach("movel")),
                          ("Ctrl+T", lambda: self._save_target())):
            sc = QAction(self)
            sc.setShortcut(QKeySequence(key))
            sc.triggered.connect(slot)
            self.addAction(sc)

        mt = QToolBar("Machining")
        mt.setObjectName("Machining")
        mt.setIconSize(QSize(22, 22))
        mt.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.addToolBarBreak()
        self.addToolBar(mt)
        self._act(mt, "Setup", glyph_icon("S", "#6c6f7d", 22), lambda: self._add_step("setup"),
                  "Setup: work frame, stock, table angle and tool tilt")
        self._op_buttons = []
        for cat, colour in catalog.CATEGORIES:
            if cat in ("Robot", "Setup"):
                continue
            b = QToolButton()
            b.setText(cat)
            b.setIcon(glyph_icon(cat[:2], colour, 22))
            b.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
            b.setPopupMode(QToolButton.InstantPopup)
            b.setMenu(QMenu(b))
            mt.addWidget(b)
            self._op_buttons.append((cat, b))
        self.roadmap_box = QCheckBox("Show roadmap")
        self.roadmap_box.setToolTip("Also list operations that have no toolpath generator yet")
        self.roadmap_box.toggled.connect(lambda *_: self._fill_operations())
        mt.addWidget(self.roadmap_box)
        mt.addSeparator()
        self.stock_box = QCheckBox("Stock")
        self.stock_box.setToolTip("Cut a voxel (SDF) stock along the simulated robot motion after each simulation; "
                                  "the bar then shows the stock as cut up to that sample")
        self.stock_box.toggled.connect(self._toggle_stock)
        mt.addWidget(self.stock_box)
        self._act(mt, "Process check", glyph_icon("F", "#6a1b9a", 22), self._process_check,
                  "Forces, robot deflection, wire bow and a voxel FEA strength check of the cut stock")
        self._act(mt, "Setup sheet", glyph_icon("Sh", "#455a64", 22), self._setup_sheet,
                  "Write a Markdown sheet of the generated program: tool, feed, spindle, path lengths, time and, with "
                  "the Stock box on, the removed volume per step")
        self.collisions = QCheckBox("Check collisions")
        try:
            import tesseract_robotics  # noqa: F401
            self.collisions.setChecked(True)
            self.collisions.setToolTip("Plan with all IK branches and check collisions on the cell meshes "
                                       "(Tesseract); slower")
        except ImportError:
            self.collisions.setEnabled(False)
            self.collisions.setToolTip("pip install tesseract-robotics to plan with collision checks")
        mt.addWidget(self.collisions)
        self.table_box = QCheckBox("Turntable")
        self.table_box.toggled.connect(self._toggle_table)
        mt.addWidget(self.table_box)
        self.post_box = QComboBox()
        self.post_box.setToolTip("Controller language of the posted program; only the cell's own controller and "
                                 "the neutral CL text are offered")
        mt.addWidget(QLabel("  Post "))
        mt.addWidget(self.post_box)
        self._fill_operations()

    DIALECT_NAMES = {"urscript": "Universal Robots URScript", "krl": "KUKA KRL", "rapid": "ABB RAPID",
                     "gcode": "ISO G-code (CNC machine file)", "cl": "Neutral CL (test)"}
    GENERATING = {"wire_ruled", "disc_cut", "disc_comb", "disc_ruled", "polish"}

    def _ready(self, spec) -> bool:
        """An operation that produces motion or a real record (not only a comment)."""
        if spec.status == "done":
            return True
        return spec.status == "partial" and (spec.id in cam.GENERATORS or spec.id in self.GENERATING)

    def _fill_operations(self):
        """Machining menus and the palette list the operations that work; the rest only with 'Show roadmap'."""
        allp = self.roadmap_box.isChecked()
        for cat, b in self._op_buttons:
            m = b.menu()
            m.clear()
            for s in catalog.steps_in(cat):
                if not (allp or self._ready(s)):
                    continue
                tag = "" if self._ready(s) else "  (roadmap)"
                a = m.addAction(step_icon(s), s.label + tag)
                a.triggered.connect(lambda *_, sid=s.id: self._add_step(sid))
            b.setEnabled(not m.isEmpty())
        while self.palette.count():
            w = self.palette.widget(0)
            self.palette.removeItem(0)
            w.deleteLater()
        for cat, _ in catalog.CATEGORIES:
            specs = [s for s in catalog.steps_in(cat) if allp or self._ready(s)]
            if not specs:
                continue
            lst = PaletteList(specs)
            lst.itemDoubleClicked.connect(lambda it: self._add_step(it.data(ROLE)))
            self.palette.addItem(lst, cat)

    def _fill_dialects(self):
        native = program.ROBOTS[self.station.robot][2]
        self.post_box.blockSignals(True)
        self.post_box.clear()
        for d in (native, "gcode", "cl"):
            self.post_box.addItem(self.DIALECT_NAMES.get(d, d), d)
        self.post_box.setCurrentIndex(0)
        self.post_box.blockSignals(False)

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
        self._fill_dialects()
        self.robot_panel.combo.blockSignals(True)
        self.robot_panel.combo.setCurrentIndex(self.robot_panel.combo.findData(name))
        self.robot_panel.combo.blockSignals(False)
        self.robot_panel.set_robot(self.robot, program.HOME.get(name, [0.0] * self.robot.dof))
        if (name.startswith("kr") or name == "irb6700") and first is False:
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
        if self.view.win.gizmo_on and not self.view.win._dragging:
            self.view.set_gizmo(self.robot.tcp(q), self._gizmo_axes())

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
        self.props.targets = st.targets
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
        tgs = QTreeWidgetItem(root, ["Targets ({})".format(len(st.targets))])
        tgs.setIcon(0, glyph_icon("+", "#00897b", 20))
        tgs.setToolTip(0, "Saved robot positions. Ctrl+T saves the current one; double-click moves the robot there")
        for t in st.targets:
            ti = QTreeWidgetItem(tgs, ["{}   {:.1f}, {:.1f}, {:.1f} mm".format(
                t["name"], *(np.asarray(t["pose"])[:3, 3] * 1000))])
            ti.setData(0, ROLE + 1, "target")
            ti.setData(0, ROLE + 2, t["name"])
            ti.setIcon(0, glyph_icon("T", "#00897b", 20))
        tgs.setExpanded(True)
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
                why = self._feed_missing(s)
                sf = self.step_feeds.get(id(s))
                if why:                                            # visible before Generate
                    si.setForeground(0, QBrush(QColor("#e0a020")))
                    si.setText(0, s.title() + "   (no feed)")
                    si.setToolTip(0, why)
                elif sf is not None:                               # the stock run exists: the feed against its limit
                    over = sf.limit.feed is not None and sf.programmed > sf.limit.feed
                    si.setText(0, "{}   [{}]".format(s.title(), sf.badge()))
                    si.setForeground(0, QBrush(QColor("#e0a020" if over else "#4caf50" if sf.limit.feed else
                                                      "#9e9e9e")))
                    si.setToolTip(0, sf.tooltip())
                if s is select:
                    sel_item = si
            pi.setExpanded(True)
        for it in (root, rob, work, progs):
            it.setExpanded(True)
        self.tree.blockSignals(False)
        self._update_checklist()
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
                    (lc.has_cutter(t) or sid not in cam.GENERATORS or
                                                                          sid in catalog.NO_CUTTER_MODEL)]
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
        if it is not None and it.data(0, ROLE + 1) == "target":
            name = it.data(0, ROLE + 2)
            m.addAction("Move robot here", lambda: self._goto_target(name))
            m.addAction("Add joint move to it", lambda: self._target_step("movej", name))
            m.addAction("Add linear move to it", lambda: self._target_step("movel", name))
            m.addAction("Update from current position", lambda: self._save_target(name))
            m.addAction("Rename", lambda: self._rename_target(name))
            m.addAction("Delete target", lambda: self._delete_target(name))
            m.exec(self.tree.viewport().mapToGlobal(pos))
            return
        if isinstance(step, program.Step):
            m.addAction("Delete step", lambda: self._delete_step(step))
            m.addAction("Duplicate step", lambda: self._dup_step(step))
        if it is not None and it.data(0, ROLE + 1) == "program":
            p = it.data(0, ROLE + 2)
            m.addAction("Simulate this program", lambda: self._simulate(prog=p))
            m.addAction("Post this program", lambda: self._post(prog=p))
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
        it = self.tree.currentItem()
        if it is not None and it.parent() is None:                  # the Station root
            self.props.show_station(self.station, self._station_edited, self)
            return
        self.props.show_step(self._current_step())

    CHECKLIST = (("cell", "1  Cell: robot, spindle, table"), ("part", "2  Part loaded"),
                 ("frame", "3  Work frame within reach"), ("stock", "4  Stock defined"),
                 ("feeds", "5  Material and feeds"), ("ops", "6  Operations added"),
                 ("gen", "7  Generated (F5)"), ("check", "8  Reach and collisions checked"),
                 ("post", "9  Posted (Ctrl+P)"))

    def _update_checklist(self):
        if not hasattr(self, "checklist"):
            return
        st = self.station
        steps = [s for p in st.programs for s in p.steps]
        cutting = [s for s in steps if s.kind in cam.GENERATORS or s.kind in self.GENERATING]
        ex = self.expansion
        fresh = ex is not None and getattr(self, "_expanded_prog", None) is not None and \
            ex.key == program.station_key(st, self._expanded_prog)
        frame_ok = None
        if st.stock is not None:
            s = st.stock
            T = program.down_pose(st.work_frame(), [(s["xmin"] + s["xmax"]) / 2, (s["ymin"] + s["ymax"]) / 2,
                                                    s["zmax"] + 20.0])
            _, frame_ok = program.solve_start(self.robot, T, self.robot_panel.q, tries=4)
        state = {"cell": True, "part": self.part is not None or bool(st.ruled_path),
                 "frame": frame_ok, "stock": st.stock is not None,
                 "feeds": (not any(self._feed_missing(s) for s in steps)) if cutting else None,
                 "ops": bool(cutting), "gen": fresh,
                 "check": (fresh and not ex.errors and ex.unreachable == 0) if fresh else None,
                 "post": fresh and self._posted_key == ex.key}
        tips = {"frame": "Select the Station item and move the work frame; the stock top centre must be reachable",
                "feeds": "A step marked 'no feed' needs a feed (or a material with cutting data)",
                "check": "See the Issues tab", "part": "Part button on the toolbar, or open an example",
                "ops": "Machining toolbar or the Operations palette", "gen": "Generate (F5) after every change",
                "post": "Post (Ctrl+P) writes the controller program"}
        self.checklist.clear()
        for key, text in self.CHECKLIST:
            v = state[key]
            it = QListWidgetItem(("✔  " if v else ("!  " if v is False and key in ("frame", "feeds", "check")
                                                      else "·  ")) + text)
            it.setForeground(QBrush(QColor("#4caf50" if v else ("#e0a020" if v is False and key in
                                                                   ("frame", "feeds", "check") else "#9aa0a6"))))
            it.setData(ROLE, key)
            it.setToolTip(tips.get(key, ""))
            self.checklist.addItem(it)

    def _checklist_go(self, it):
        key = it.data(ROLE)
        if key == "part":
            self._load_part()
        elif key in ("frame", "stock", "feeds"):
            self.tree.setCurrentItem(self.tree.topLevelItem(0))
        elif key == "gen":
            self._simulate()
        elif key in ("check",):
            self.bottom_tabs.setCurrentWidget(self.issues)
        elif key == "post":
            self._post()

    def _show_issues(self, ex):
        self.issues.clear()
        keys = ("unreachable", "collid", "did not", "uncut", "WARNING", "out of reach", "no feed", "missing",
                "faces down", "left uncut", "exceeds")
        n_err = 0
        for e in ex.errors:
            it = QListWidgetItem("Error   " + e)
            it.setForeground(QBrush(QColor("#ef5350")))
            self.issues.addItem(it)
            n_err += 1
        for m in ex.messages:
            if m.startswith("ERROR"):
                continue
            if any(k in m for k in keys):
                it = QListWidgetItem("Check   " + m)
                it.setForeground(QBrush(QColor("#e0a020")))
                self.issues.addItem(it)
        if ex.unreachable:
            self.issues.addItem(QListWidgetItem("Check   {} robot samples are unreachable or colliding (red in the "
                                                "toolpath)".format(ex.unreachable)))
        if not self.issues.count():
            self.issues.addItem(QListWidgetItem("No issues"))
        self.bottom_tabs.setTabText(0, "Issues ({})".format(self.issues.count() if self.issues.item(0).text() !=
                                                             "No issues" else 0))
        if n_err or ex.unreachable:
            self.bottom_tabs.setCurrentWidget(self.issues)

    def _feed_missing(self, s) -> str:
        """Why a cutting step would be refused at post time for lack of a feed ('' when a feed exists)."""
        if s.kind not in cam.GENERATORS and s.kind not in ("disc_cut", "disc_comb", "disc_ruled"):
            return ""
        if s.kind == "tap":
            return ""                                  # the tap feed is pitch x rpm, set in the move
        if float(s.params.get("feed") or 0) > 0:
            return ""
        t = self._tools().get(str(s.params.get("tool") or ""))
        if t is None:
            return "no tool"
        if float(getattr(t, "feed", 0) or 0) > 0:
            return ""
        try:
            from .. import cutting_data
            sp = self.station.spindle_model()
            if cutting_data.suggest(t, self.station.material, float(s.params.get("stepdown", 0) or 0), sp.max_rpm,
                                    self.station.robot_factor).feed > 0:
                return ""
        except Exception:   # noqa: BLE001
            pass
        return ("No feed: the step, tool {} and the cutting data for {} give none. Set the step's feed; "
                "posting is refused until then.".format(t.id, self.station.material))

    def _table_mode_changed(self, mode):
        """The table is driven differently: the planning model (the collision backend with or without a table joint)
        is built again on the next simulation, so the present expansion is stale."""
        self.say("Table driven by '{}': simulate again; the collision backend is rebuilt from the station".format(mode))
        self._posted_key = None
        self._clear_stock()
        self._update_checklist()

    def _station_edited(self):
        self._refresh_scene()
        self._update_checklist()

    def _step_edited(self):
        it = self.tree.currentItem()
        step = self._current_step()
        if it is not None and step is not None:
            it.setText(0, step.title())

    # ---------------------------------------------------------------- targets and the TCP handle
    def _save_target(self, name=""):
        tool = self._first_tool()
        t = self.station.add_target(self.robot_panel.q, self.robot.tcp(self.robot_panel.q),
                                    getattr(tool, "id", "") if tool is not None else "", name)
        self.props.targets = self.station.targets
        self._rebuild_tree()
        self.say("Saved target {} at {:.1f}, {:.1f}, {:.1f} mm".format(t["name"], *(np.asarray(t["pose"])[:3, 3] *
                                                                                    1000)))
        return t

    def _goto_target(self, name):
        t = self.station.target(name)
        if t is not None:
            self.robot_panel.set_q(np.radians(np.asarray(t["joints"], dtype=float)))
            self.say("Robot at target {}".format(name))

    def _target_step(self, kind, name):
        step = program.Step(kind, dict(catalog.default_params(catalog.BY_ID[kind]), target=name))
        prog = self._current_program()
        cur = self._current_step()
        if cur is not None and cur in prog.steps:
            prog.steps.insert(prog.steps.index(cur) + 1, step)
        else:
            prog.steps.append(step)
        self._rebuild_tree(select=step)
        self.say("Added {} to target {}".format(catalog.BY_ID[kind].label.lower(), name))

    def _rename_target(self, name):
        from PySide6.QtWidgets import QInputDialog
        new, ok = QInputDialog.getText(self, "Rename target", "Name", text=name)
        new = new.strip()
        if not ok or not new or new == name:
            return
        if self.station.target(new) is not None:
            self.say("A target named {} exists already".format(new))
            return
        self.station.target(name)["name"] = new
        for p in self.station.programs:          # steps follow the rename
            for s in p.steps:
                if s.params.get("target") == name:
                    s.params["target"] = new
        self._rebuild_tree()

    def _delete_target(self, name):
        used = [s for p in self.station.programs for s in p.steps if s.params.get("target") == name]
        if used and QMessageBox.question(self, "Delete target", "{} steps use {}. Delete it anyway? They will keep "
                                         "their stored values.".format(len(used), name)) != QMessageBox.Yes:
            return
        t = self.station.target(name)
        for s in used:                           # keep the motion: copy the values into the step
            s.params.pop("target", None)
            s.params["joints"], s.params["pose"] = t["joints"], t["pose"]
        self.station.targets.remove(t)
        self._rebuild_tree()

    def _gizmo_axes(self):
        f = self.robot_panel.frame.currentText().lower()
        if f == "tool":
            return self.robot.tcp(self.robot_panel.q)[:3, :3]
        if f == "work":
            return self.station.work_frame()[:3, :3]
        return np.eye(3)

    def _toggle_gizmo(self, on):
        self.view.show_gizmo(on)
        if on:
            self.view.set_gizmo(self.robot.tcp(self.robot_panel.q), self._gizmo_axes())
            self.say("Drag the arrows to move along an axis, the rings to rotate, the centre square to move in the "
                     "screen plane. Hold Shift to snap to the jog step. Ctrl+T saves the position as a target.")

    def _gizmo_moved(self, F, final):
        q, ok = self.robot.solve_pose(F, self.robot_panel.q)
        jump = float(np.max(np.abs(np.asarray(q) - self.robot_panel.q))) if ok else 0.0
        if ok and jump < math.radians(30):         # refuse a branch flip in the middle of a drag
            self.robot_panel.set_q(q)
        elif final:
            self.say("Out of reach or at a joint limit; the robot stays at the last reachable pose")
        if final and self.view.win.gizmo_on:
            self.view.set_gizmo(self.robot.tcp(self.robot_panel.q), self._gizmo_axes())

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
            program.Step("comment", {"text": "Demo feeds are example values for a soft material; set the feeds "
                                             "for your stone and tools before posting"}),
            program.Step("rough_plane", {"tool": "W2", "stepdown": 8.0, "stepover": 4.0, "allowance": 1.0,
                                         "feed": 600.0}),
            program.Step("raster", {"tool": "W7", "stepover": 2.0, "feed": 600.0}),
            program.Step("movej", {"joints": [math.degrees(v) for v in program.HOME[self.station.robot]]}),
        ])]
        self._mount(self._first_tool())
        self._refresh_scene()
        self._rebuild_tree()
        self.say("Demo station: a dome on a {:.0f} mm stock; the program roughs, finishes and lists a polish step. "
                 "Press Simulate.".format(size + 10))

    def _load_part(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Part", "", "Part (*.stl *.npz);;STL (*.stl);;Height field (*.npz)")
        if not fn:
            return
        try:
            tris = cam.load_part(fn)
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

    def _scene_work(self) -> np.ndarray:
        """The work frame turned with the table (the part's frame in the scene)."""
        W0 = self.station.work_frame()
        yaw = getattr(self, "_yaw", 0.0)
        W = W0.copy()
        W[:3, :3] = W0[:3, :3] @ np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0],
                                           [0, 0, 1.0]])
        return W

    def _refresh_scene(self):
        W = self._scene_work()
        self.view.set_part(self.part_tris, W)
        self.view.set_stock(self.station.stock, W)
        self.view.set_table(self.station.turntable, W)
        self.robot_panel.work = W
        if getattr(self, "stock_run", None) is not None:      # the cut stock turns with the table too
            self._show_stock(self.sim.value(), force=True)
        self._robot_moved(self.robot_panel.q)

    def _toggle_table(self, on):
        self.station.turntable = {"diameter": 0.8, "height": 0.04} if on else None
        self._yaw = 0.0
        self._refresh_scene()

    def _work_centre(self):
        """Centre of the stock (else the part, else the work frame origin) in the robot base frame, metres, and the
        stock diagonal in metres (0 when there is neither)."""
        W = self.station.work_frame()
        s = self.station.stock
        if isinstance(s, dict) and all(k in s for k in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")):
            lo = np.array([s["xmin"], s["ymin"], s["zmin"]], dtype=float)
            hi = np.array([s["xmax"], s["ymax"], s["zmax"]], dtype=float)
        elif self.part_tris is not None:
            p = np.asarray(self.part_tris, dtype=float).reshape(-1, 3)
            lo, hi = p.min(0), p.max(0)
        else:
            return W[:3, 3].copy(), 0.0
        c = (lo + hi) / 2000.0                                  # mm -> m, work frame
        return W[:3, :3] @ c + W[:3, 3], float(np.linalg.norm(hi - lo)) / 1000.0

    def _fit_part_and_robot(self):
        """Camera on the cut: centred between the stock and the robot's reach, close enough to see the part."""
        w, _diag = self._work_centre()
        q = self.sim_q[len(self.sim_q) // 2] if len(self.sim_q) else self.robot_panel.q
        tf = self.robot.link_transforms(q)
        pts = np.array([T[:3, 3] for T in tf.values()] + [w])
        lo, hi = pts.min(0), pts.max(0)
        win = self.view.win
        win.center = 0.35 * (lo + hi) / 2 + 0.65 * w
        win.dist = max(0.6, 1.15 * float(np.linalg.norm(hi - lo)))
        win.yaw, win.pitch = math.radians(-60), math.radians(28)
        win.update_camera()

    def _view(self, which):
        tf = self.robot.link_transforms(self.robot_panel.q)
        pts = [T[:3, 3] for T in tf.values()] + [self.station.work_frame()[:3, 3]]
        lo, hi = np.min(pts, 0) - 0.15, np.max(pts, 0) + 0.15
        self.view.win.view(which or "keep", (lo, hi))
        self.view.win.center = (self.view.win.center + self.station.work_frame()[:3, 3]) / 2
        self.view.win.dist *= 0.95
        self.view.win.update_camera()

    # ---------------------------------------------------------------- simulate and post
    def _simulate(self, *_args, prog=None):
        prog = prog or self._current_program()
        self._expanded_prog = prog
        self.say("Expanding '{}' on the {} ...".format(prog.name, program.ROBOTS[self.station.robot][0]))
        import copy
        from . import livestock
        new_sig = livestock.signature(self.station, prog)
        live = None
        if self.stock_run is not None and self.expansion is not None and getattr(self, "_exp_sig", None):
            # a rest step gets the stock of the last simulation just before it, while the steps before it are unchanged
            live = livestock.LiveStock(self.stock_run, self.expansion, self._tools(), copy.deepcopy(self.station),
                                       program.load_robot(self.station.robot), self._exp_sig, new_sig)
        self._pending_sig = new_sig
        th = QThread(self)
        wk = Expander(self.station, program.load_robot(self.station.robot), prog, self._tools(), self.part,
                      self.robot_panel.q, self.collisions.isChecked(), live)
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
        if err or ex is None or not len(ex.joints()):
            self.say("Expansion failed:\n" + (err or "nothing to simulate"))
            if getattr(self, "record", None) or getattr(self, "smoke", None):   # batch modes never hang
                out = self.record[0] if getattr(self, "record", None) else str(Path(self.smoke[0]).parent)
                Path(out).mkdir(parents=True, exist_ok=True)
                Path(out, "log.txt").write_text(self.log.toPlainText())
                QApplication.exit(3)
            return
        self.expansion = ex
        self._exp_sig = getattr(self, "_pending_sig", None)    # what this expansion was made from (livestock)
        self._clear_stock()                 # the cut stock belongs to the previous expansion
        for m in ex.messages:
            self.say("  " + m)
        self._show_issues(ex)
        self._posted_key = None
        self._update_checklist()
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
        self.traj.set_data(q, self.robot.lower, self.robot.upper, list(self.sim_seg_end[:-1]), feed)
        self.sim.blockSignals(True)
        self.sim.setRange(0, len(q) - 1)
        self.sim.setValue(0)
        self.sim.blockSignals(False)
        self._scrub(0)
        self.say("Simulation ready: {} robot samples, {} unreachable. Press play or drag the bar.".format(
            len(q), ex.unreachable))
        self._post(show_only=True)
        if self.stock_box.isChecked():
            if getattr(self, "record", None):
                self._after_stock = self._record_after_stock
            if self._start_stock() and getattr(self, "record", None):
                return                           # the recording starts when the stock run is in
            self._after_stock = None
        if getattr(self, "record", None):
            QTimer.singleShot(800, self._record_frames)
        elif getattr(self, "smoke", None):
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

    def _record_frames(self):
        """--record DIR N: N evenly spaced frames of the simulation, the 3D view only, as DIR/frame_000.png ...
        (docs/media/frames_to_gif.py turns them into a GIF)."""
        out, n = self.record
        Path(out).mkdir(parents=True, exist_ok=True)
        for d in self.findChildren(QDockWidget):          # the 3D view alone, as large as the window
            d.hide()
        for tb in self.findChildren(QToolBar):
            tb.hide()
        QApplication.processEvents()
        self._fit_part_and_robot()
        if self.stock_run is not None:          # the cut stock is the subject: closer, centred on the work
            win = self.view.win
            c, diag = self._work_centre()
            win.center = 0.25 * win.center + 0.75 * c
            win.dist = max(0.5, 3.0 * diag) if diag > 0 else win.dist * 0.45
            win.update_camera()
        idx = [int(round(v)) for v in np.linspace(0, self.sim.maximum(), n)]
        state = {"k": 0}
        cont = self.view_container

        def shot():
            k = state["k"]
            if k >= len(idx):
                Path(out, "log.txt").write_text(self.log.toPlainText())
                QApplication.quit()
                return
            self.sim.setValue(idx[k])
            QApplication.processEvents()

            def grab():
                tl = cont.mapToGlobal(cont.rect().topLeft())
                pm = QApplication.primaryScreen().grabWindow(0, tl.x(), tl.y(), cont.width(), cont.height())
                pm.save(str(Path(out) / "frame_{:03d}.png".format(k)))
                state["k"] += 1
                QTimer.singleShot(30, shot)
            QTimer.singleShot(120, grab)
        shot()

    def _demo_five_axis(self):
        """Built-in example: the dome, a raster pass and a 5-axis ball surfacing pass."""
        self._demo()
        prog = self.station.programs[0]
        ball = next((t for t in self.tool_lib if t.kind == lc.ToolKind.BallEndMill and 4 <= t.diameter <= 12), None)
        if ball is not None:
            prog.steps.insert(len(prog.steps) - 1,
                              program.Step("surfacing5", {"tool": ball.id, "stepover": 4.0, "feed": 600.0}))
        self._rebuild_tree()
        self.say("Example: dome with raster and 5-axis surfacing. Press Simulate.")

    def _teach_smoke(self, out):
        """--teach-smoke: turn the TCP handle on, drag the z arrow 80 mm down and the z ring 30 deg through the same
        code path as the mouse, save two targets, add a move to each, capture, quit."""
        self.act_gizmo.setChecked(True)
        self._toggle_gizmo(True)
        win = self.view.win
        QApplication.processEvents()
        cam = win.camera_model()
        g = win.gizmo
        L = g.length(cam)
        o = g.frame[:3, 3]
        self._save_target("approach")
        for k, delta in ((2, -0.08),):
            a = g.handle_frame()[:3, k]
            p0, _ = cam.project(o + a * L * 0.7)
            p1, _ = cam.project(o + a * (L * 0.7 + delta))
            assert g.begin(cam, *p0), "the z arrow was not picked"
            F = g.drag(cam, *p1)
            self._gizmo_moved(F, True)
            g.end()
        self._save_target("cut_start")
        self._target_step("movej", "approach")
        self._target_step("movel", "cut_start")
        moved = np.linalg.norm(np.asarray(self.station.target("cut_start")["pose"])[:3, 3] -
                               np.asarray(self.station.target("approach")["pose"])[:3, 3])
        self.say("teach smoke: handle drag moved the TCP {:.1f} mm".format(moved * 1000))

        def grab():
            g2 = self.frameGeometry()
            QApplication.primaryScreen().grabWindow(0, g2.x(), g2.y(), g2.width(), g2.height()).save(out)
            Path(out).with_suffix(".log.txt").write_text(self.log.toPlainText())
            Path(out).with_suffix(".lcs.json").write_text(self.station.to_json())
            QApplication.quit()
        QTimer.singleShot(1200, grab)

    def _teach_record(self, out):
        """--teach-record DIR: the teach smoke as frames of the whole window. Saves a target, drags the z arrow down,
        the x arrow along and the z ring through 30 deg (the mouse code path), saves a second target, adds a joint
        and a linear move to the two, then goes to each target. DIR/frame_000.png ..."""
        Path(out).mkdir(parents=True, exist_ok=True)
        self.act_gizmo.setChecked(True)
        self._toggle_gizmo(True)
        win = self.view.win
        QApplication.processEvents()
        win.center = self.robot.tcp(self.robot_panel.q)[:3, 3].copy()     # the tool tip in the middle, closer
        win.dist *= 0.55
        win.update_camera()
        QApplication.processEvents()
        cam = win.camera_model()
        g = win.gizmo
        acts = []                                  # each entry is one frame: a callable run before the grab

        def hold(n):
            acts.extend([lambda: None] * n)

        def drag(kind, k, amount, n):
            box = {}

            def start():
                L = g.length(cam)
                H = g.handle_frame()
                o = H[:3, 3]
                if kind == "arrow":
                    a = H[:3, k]
                    box["pt"] = lambda t: cam.project(o + a * (L * 0.7 + amount * t))[0]
                else:
                    i, j = [m for m in range(3) if m != k]
                    R = L * g.RING
                    box["pt"] = lambda t: cam.project(o + R * (math.cos(amount * t) * H[:3, i] +
                                                               math.sin(amount * t) * H[:3, j]))[0]
                if not g.begin(cam, *box["pt"](0.0)):
                    raise RuntimeError("teach record: the {} {} was not picked".format(kind, k))
                self.view.highlight_gizmo(g.mode)
            acts.append(start)
            for s in range(1, n + 1):
                def step(t=s / n, last=(s == n)):
                    F = g.drag(cam, *box["pt"](t))
                    if F is not None:
                        self._gizmo_moved(F, last)
                    if last:
                        g.end()
                        self.view.highlight_gizmo(None)
                acts.append(step)

        hold(4)
        acts.append(lambda: self._save_target("approach"))
        hold(3)
        drag("arrow", 2, -0.10, 14)
        hold(2)
        drag("arrow", 0, 0.12, 14)
        hold(2)
        drag("ring", 2, math.radians(30), 12)
        hold(2)
        acts.append(lambda: self._save_target("cut_start"))
        hold(3)
        acts.append(lambda: self._target_step("movej", "approach"))
        acts.append(lambda: self._target_step("movel", "cut_start"))
        hold(3)
        acts.append(lambda: self._goto_target("approach"))
        hold(5)
        acts.append(lambda: self._goto_target("cut_start"))
        hold(5)
        state = {"k": 0}

        def shot():
            k = state["k"]
            if k >= len(acts):
                Path(out, "log.txt").write_text(self.log.toPlainText())
                Path(out, "station.lcs.json").write_text(self.station.to_json())
                QApplication.quit()
                return
            try:
                acts[k]()
            except Exception as e:   # noqa: BLE001
                self.say(str(e))
                self._batch_exit_on_error()
                return
            QApplication.processEvents()

            def grab():
                g2 = self.frameGeometry()
                QApplication.primaryScreen().grabWindow(0, g2.x(), g2.y(), g2.width(), g2.height()).save(
                    str(Path(out) / "frame_{:03d}.png".format(k)))
                state["k"] += 1
                QTimer.singleShot(20, shot)
            QTimer.singleShot(90, grab)
        shot()

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
        self._turn_part(k, i)
        self.robot_panel.set_q(q, emit=False)
        self.view.update_robot(robot.link_transforms(q), robot.tcp(q))
        self.sim_label.setText("{}/{}".format(i + 1, len(self.sim_q)))
        self.traj.set_cursor(i)
        self._show_stock(i)

    def _turn_part(self, k: int, i: int):
        """Turn the part with the table to the angle of sample i: Segment.e1 holds it per sample (it changes within a
        segment when the table is driven as E1), else the segment's yaw."""
        segs = self.expansion.segments
        if k >= len(segs):
            return
        seg = segs[k]
        j = i - (int(self.sim_seg_end[k - 1]) if k > 0 else 0)
        e = float(seg.e1[j]) if seg.e1 is not None and 0 <= j < len(seg.e1) else float(seg.yaw)
        if abs(e - getattr(self, "_yaw", 0.0)) > 1e-9:
            self._yaw = e
            W = self._scene_work()
            self.view.set_part(self.part_tris, W)
            self.view.set_stock(self.station.stock, W)
            self.view.set_table(self.station.turntable, W)
            self.robot_panel.work = W
            if getattr(self, "stock_run", None) is not None:
                self._show_stock(i, force=True)

    def _stop(self):
        self.btn_play.setChecked(False)
        self.sim.setValue(0)

    def _post(self, show_only=False, prog=None):
        if self.expansion is None:
            self.say("Simulate first: the post uses the verified joint solutions")
            return
        prog = prog or self._expanded_prog
        if self.expansion.key != program.station_key(self.station, prog):
            self.say("Not posted: '{}' changed (or is not the simulated program) since the last simulation. "
                     "Simulate it again, then post.".format(prog.name))
            self.output.setPlainText("; not posted: the program changed since it was simulated\n")
            return
        if self.expansion.errors:
            self.say("Not posted: {} error(s) must be fixed first:\n  ".format(len(self.expansion.errors)) +
                     "\n  ".join(self.expansion.errors))
            self.output.setPlainText("; not posted:\n" + "\n".join("; " + e for e in self.expansion.errors) + "\n")
            return
        dialect = self.post_box.currentData()
        if dialect == "gcode":
            # a CNC machine file: tip positions and tool axes read back from the G-code by forward kinematics
            mfile = self.gcode_machine or str(_examples_dir().parent / "gcode_machines" / "generic_3axis.json")
            warn = []
            try:
                texts = post.write_files(self.expansion.records, "gcode", machine=mfile,
                                         work_frame=self.station.work_frame(), warnings=warn)
            except ValueError as e:
                self.say("Not posted (G-code, {}): {}".format(Path(mfile).name, e))
                self.output.setPlainText("; not posted: {}\n".format(e))
                return
            text = "\n".join(texts)
            chk = post.compare_gcode(self.expansion.records, texts, machine=mfile,
                                     work_frame=self.station.work_frame())
            same = chk.ok
            for w in warn:
                self.say("  " + w)
            self.output.setPlainText(text)
            self.say("Posted {} lines of G-code for {} in {} file(s); read-back: {} ({} of {} points, tip {:.4f} mm, "
                     "axis {:.3f} deg)".format(text.count("\n"), Path(mfile).name, len(texts),
                                               "PASS" if same else "FAIL", chk.matched, chk.n, chk.max_tip_err,
                                               chk.max_axis_err))
        else:
            try:
                text = post.write(self.expansion.records, dialect,
                                  **program.post_options(self.station.robot, dialect))
            except ValueError as e:
                self.say("Not posted ({}): {}".format(dialect, e))
                self.output.setPlainText("; not posted: {}\n".format(e))
                return
            self.output.setPlainText(text)
            back = post.joint_targets(text, dialect)
            rec_q = [r.data["q"] for r in self.expansion.records if r.kind in ("movej", "movel")]
            same = len(back) == len(rec_q) and (not back or np.max(np.abs(np.asarray(back) -
                                                                         np.asarray(rec_q))) < 1e-5)
            e1_note = ""
            want_e1 = [r.data.get("e1") for r in self.expansion.records if r.kind in ("movej", "movel")]
            if any(v is not None for v in want_e1):           # the table is a coordinated axis: read E1 back too
                got_e1 = post.external_axes(text, dialect)
                e1_same = len(got_e1) == len(want_e1) and all(
                    (g is None) == (w is None) and (g is None or abs(g - w) < 1e-3) for g, w in zip(got_e1, want_e1))
                same = same and e1_same
                e1_note = "; E1 read-back {} ({} values)".format("PASS" if e1_same else "FAIL",
                                                                  sum(v is not None for v in got_e1))
            self.say("Posted {} lines ({}); read-back check: {}{}".format(text.count("\n"), dialect,
                                                                            "PASS" if same else "FAIL", e1_note))
        if show_only:
            return
        if same:
            self._posted_key = self.expansion.key
            self._update_checklist()
        fn, _ = QFileDialog.getSaveFileName(self, "Save robot program", prog.name + post.EXTENSIONS[dialect])
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
                    params = program._gen_params(s.kind, dict(catalog.default_params(s.spec), **s.params))
                    try:
                        moves = cam.GENERATORS[s.kind](tris, mesh, tool, self.station.stock, **params)
                    except Exception as e:   # noqa: BLE001 - e.g. rest machining needs the previous tool
                        self.say("Z-map {} / {}: not checked ({})".format(p.name, s.title(), e))
                        continue
                    stats, _ = cam.zmap_check(tris, mesh, tool, moves, self.station.stock, 1.0)
                    self.say("Z-map {} / {}: leftover max {:.3f} mm, gouge {:.3f} mm over {} cells".format(
                        p.name, s.title(), stats.max_dev, -min(0.0, stats.min_dev), stats.cells))

    # ---------------------------------------------------------------- cut stock (studio.process)
    def _clear_stock(self):
        self._stock_gen += 1                 # a run still in flight is now stale
        self._stock_busy = False
        self.stock_run = None
        self.process_report = None
        self._stock_shown = None
        if self.step_feeds:
            self.step_feeds = {}
            self._rebuild_tree()
        self.view.set_stock_mesh(None, None, np.eye(4))
        self.view.show_part(True)
        self.view.show_path(True)

    def _toggle_stock(self, on):
        if not on:
            self._after_stock = None
            self._clear_stock()
            return
        if self.expansion is None:
            self.say("Stock: the cut stock is computed after the next simulation")
        elif self.stock_run is None and not self._stock_busy:
            self._start_stock()

    def _start_stock(self) -> bool:
        if self.station.stock is None:
            self.say("Stock: the station has no stock box")
            return False
        import copy
        self._stock_gen += 1
        self._stock_busy = True
        self.stock_run = None
        self.process_report = None
        self.say("Cutting the stock along {} robot samples ...".format(len(self.sim_q)))
        th = QThread(self)
        wk = StockWorker(self._stock_gen, copy.deepcopy(self.station), program.load_robot(self.station.robot),
                         self.expansion, self._tools())
        wk.moveToThread(th)
        th.started.connect(wk.run)
        wk.progress.connect(self._progress)      # bound slots: queued into the GUI thread
        wk.done.connect(self._stock_done)
        wk.done.connect(th.quit)
        self._threads.append((th, wk))
        th.start()
        return True

    def _stock_done(self, gen, run, err):
        if gen != self._stock_gen:
            return
        self._stock_busy = False
        if err or run is None:
            self.say("Stock simulation failed:\n" + (err or "no result"))
            self._batch_exit_on_error()
            return
        self.stock_run = run
        for m in run.messages:
            self.say("  " + m)
        self._update_step_feeds()
        self._stock_shown = None
        if self.stock_box.isChecked():
            self._show_stock(self.sim.value(), force=True)
        cb, self._after_stock = self._after_stock, None
        if cb is not None:
            cb()

    def _update_step_feeds(self, feeds=None):
        """Feed-limit badges in the tree: from a process report, or from the stock run with the sourced constants."""
        self.step_feeds = {}
        prog = getattr(self, "_expanded_prog", None)
        if prog is None or self.stock_run is None or self.expansion is None:
            return
        try:
            if feeds is None:
                from . import process
                feeds = process.step_feed_limits(self.station, self.stock_run, self.expansion, self.robot, self._tools(),
                                                 prog, self._constants_given)
            for idx, sf in feeds.items():
                if idx < len(prog.steps):
                    self.step_feeds[id(prog.steps[idx])] = sf
        except Exception as e:   # noqa: BLE001 - the badges are advisory
            self.say("Feed limits not shown: {}".format(e))
        self._rebuild_tree()

    def _show_stock(self, i, force=False):
        """The latest stock snapshot at or before sample i; stress colours on the final one after a process check."""
        run = self.stock_run
        if run is None or not self.stock_box.isChecked():
            return
        k = run.frame_at(int(i))
        if k == self._stock_shown and not force:
            return
        self._stock_shown = k
        _, V, F = run.frames[k]
        rep = self.process_report
        colors = rep.colors if rep is not None and rep.colors is not None and k == len(run.frames) - 1 else None
        self.view.set_stock_mesh(V, F, self._scene_work(), colors)
        self.view.show_part(False)               # the cut stock is the part being made; the model would z-fight
        self.view.show_path(False)               # and it shows where the tool has been

    def _setup_sheet(self):
        """Save studio.sheet.setup_sheet of the generated program (with removed volumes when the stock run is in)."""
        if self.expansion is None:
            self.say("Setup sheet: generate the program first (F5)")
            return
        from . import sheet
        prog = self._expanded_prog
        text = sheet.setup_sheet(self.station, self.expansion, prog, self._tools(), self.stock_run)
        fn, _ = QFileDialog.getSaveFileName(self, "Save setup sheet", prog.name + "_setup_sheet.md", "Markdown (*.md)")
        if not fn:
            return
        Path(fn).write_text(text, encoding="utf-8")
        self.say("Setup sheet written to {}{}".format(fn, "" if self.stock_run is not None else
                                                         " (no removed volumes: switch the Stock box on first)"))

    def _process_check(self):
        if self.expansion is None:
            self.say("Process check: simulate first")
            return
        if self.stock_run is None:
            self._after_stock = self._process_check
            self.say("Process check: cutting the stock first")
            if not self.stock_box.isChecked():
                self.stock_box.setChecked(True)  # starts the stock run
            elif not self._stock_busy:
                self._start_stock()
            return
        dlg = ConstantsDialog(self, self.station.material, self.station.robot, self._constants_given)
        if dlg.exec() != QDialog.Accepted:
            return
        try:
            given = dlg.values()
        except ValueError as e:
            QMessageBox.warning(self, "Process check", str(e))
            return
        self._constants_given = given
        self._start_process(given)

    def _start_process(self, given):
        import copy
        self.say("Process check ...")
        th = QThread(self)
        wk = ProcessWorker(self._stock_gen, copy.deepcopy(self.station), program.load_robot(self.station.robot),
                           self.stock_run, self.expansion, self._tools(), given,
                           copy.deepcopy(getattr(self, "_expanded_prog", None)))
        wk.moveToThread(th)
        th.started.connect(wk.run)
        wk.progress.connect(self._progress)
        wk.done.connect(self._process_done)
        wk.done.connect(th.quit)
        self._threads.append((th, wk))
        th.start()

    def _process_done(self, gen, rep, err):
        if err or rep is None:
            self.say("Process check failed:\n" + (err or "no result"))
            self._batch_exit_on_error()
            return
        if gen != self._stock_gen:
            return
        self.process_report = rep
        if rep.feed_limits:
            self._update_step_feeds(rep.feed_limits)
        text = rep.text()
        self.say(text)
        self.process_text.setPlainText(text)
        self.bottom_tabs.setCurrentWidget(self.process_text)
        if rep.colors is not None:
            self.say("Stress colours are on the final stock: the bar is at the end")
            self.sim.setValue(self.sim.maximum())
            self._stock_shown = None
            self._show_stock(self.sim.value(), force=True)
        cb, self._after_process = self._after_process, None
        if cb is not None:
            cb()

    def _record_after_stock(self):
        """--record with --stock: optionally run the process check (--process), then record the frames."""
        if getattr(self, "record_process", None) is not None:
            self._after_process = lambda: QTimer.singleShot(800, self._record_frames)
            self._start_process(self.record_process)
        else:
            QTimer.singleShot(800, self._record_frames)

    def _batch_exit_on_error(self):
        if getattr(self, "record", None):
            Path(self.record[0]).mkdir(parents=True, exist_ok=True)
            Path(self.record[0], "log.txt").write_text(self.log.toPlainText())
            QApplication.exit(3)

    # ---------------------------------------------------------------- files
    def _new(self):
        self.station = program.Station(robot=self.station.robot)
        self.part = self.part_tris = None
        self.expansion = None
        self._clear_stock()
        self.view.set_path(None, None)
        self._refresh_scene()
        self._rebuild_tree()

    def _open(self):
        fn, _ = QFileDialog.getOpenFileName(self, "Open station", "", "lithocam station (*.lcs.json *.json)")
        if fn:
            self.open_station(fn)

    def open_station(self, fn):
        try:
            st = program.Station.load(fn)
            self.station = st
            if st.tool_library:
                self.tool_lib = list(lc.read_tool_library(st.tool_library).tools)
                self._tools_changed()
            self.part_tris, self.part = None, None
            if st.part_path and Path(st.part_path).exists():
                tris = cam.load_part(st.part_path)          # STL or height-field .npz
                self.part_tris, self.part = tris, (tris, cam.trimesh(tris))
                if st.stock is None:
                    st.stock = cam.stock_box(tris, 0.0)
            self._yaw = 0.0
            self.expansion = None
            self._clear_stock()
            self.view.set_path(None, None)
            self.table_box.blockSignals(True)
            self.table_box.setChecked(st.turntable is not None)
            self.table_box.blockSignals(False)
            self._set_robot(st.robot, first=True)
            self._view("iso")
            self.say("Opened {}".format(fn))
        except Exception as e:   # noqa: BLE001
            if getattr(self, "batch", False):               # no dialog in batch modes: log it and exit
                self.say("Open {} failed: {}".format(fn, e))
                print("Open {} failed: {}".format(fn, e), file=sys.stderr)
                QTimer.singleShot(0, lambda: QApplication.exit(3))
                return
            QMessageBox.warning(self, "Open", str(e))

    def _save(self):
        fn, _ = QFileDialog.getSaveFileName(self, "Save station", "station.lcs.json", "lithocam station (*.json)")
        if fn:
            Path(fn).write_text(self.station.to_json())
            self.say("Saved {}".format(fn))


def _examples_dir() -> Path:
    """examples/studio of the repo, or of the bundle in a frozen executable."""
    if getattr(sys, "frozen", False):
        return Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent)) / "examples" / "studio"
    return program.CELLS.parent / "examples" / "studio"


def main(argv=None) -> int:
    """Options: --robot ur10e|kr120|kr480_r3330_mt|kr500_r2830_mt|irb6700; --smoke OUT.png [FRACTION]: simulate the demo, capture, exit (CI and docs);
    --record DIR N [--five-axis-demo] [--no-collisions] [--stock] [--process [--constants k=v,...]] [--timeout S]:
    N frames of the simulation (with --stock the cut stock, with --process stress colours on the final stock)."""
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
    stations = [a for a in argv[1:] if a.endswith(".json")]
    w.batch = any(f in argv for f in ("--record", "--smoke", "--teach-smoke", "--teach-record"))
    if stations:
        w.open_station(stations[0])
    w.show()
    w.showMaximized()
    if "--record" in argv:
        k = argv.index("--record")
        n = int(argv[k + 2]) if len(argv) > k + 2 and argv[k + 2].isdigit() else 60
        w.record = (argv[k + 1], n)
        if "--five-axis-demo" in argv:
            w._demo_five_axis()
        if "--no-collisions" in argv:
            w.collisions.setChecked(False)
        if "--stock" in argv or "--process" in argv:     # show the stock being cut
            w.stock_box.setChecked(True)
        if "--process" in argv:
            # constants: sourced rows for the station material and robot, plus any given as
            # --constants key=value,key=value (a key given as key= refuses its sourced row)
            given = {}
            if "--constants" in argv:
                for kv in argv[argv.index("--constants") + 1].split(","):
                    if "=" in kv:
                        k_, v_ = kv.split("=", 1)
                        given[k_.strip()] = float(v_) if v_.strip() else None
            w.record_process = given
        limit =int(argv[argv.index("--timeout") + 1]) if "--timeout" in argv else 900
        QTimer.singleShot(limit * 1000, lambda: (Path(w.record[0]).mkdir(parents=True, exist_ok=True),
                                                 Path(w.record[0], "log.txt").write_text(
                                                     w.log.toPlainText() + "\nTIMEOUT"), QApplication.exit(4)))
        # Backstop: a long native call holds the interpreter lock and the Qt timer above cannot run. faulthandler's
        # watchdog runs without it; it writes every thread's stack to DIR/timeout_stack.txt and exits.
        Path(w.record[0]).mkdir(parents=True, exist_ok=True)
        w._watchdog_file = open(Path(w.record[0], "timeout_stack.txt"), "w")
        faulthandler.dump_traceback_later(limit + 60, exit=True, file=w._watchdog_file)
        QTimer.singleShot(1500, w._simulate)
    elif "--teach-record" in argv:
        out = argv[argv.index("--teach-record") + 1]
        w.record = (out, 0)                               # for _batch_exit_on_error
        QTimer.singleShot(240 * 1000, lambda: QApplication.exit(4))
        QTimer.singleShot(1500, lambda: w._teach_record(out))
    elif "--teach-smoke" in argv:
        out = argv[argv.index("--teach-smoke") + 1]
        QTimer.singleShot(1500, lambda: w._teach_smoke(out))
    elif "--smoke" in argv:
        k = argv.index("--smoke")
        frac = float(argv[k + 2]) if len(argv) > k + 2 and not argv[k + 2].startswith("--") else 0.6
        w.smoke = (argv[k + 1], frac)
        QTimer.singleShot(1500, w._simulate)
    return app.exec()
