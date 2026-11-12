# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Joint-trajectory plot (in the spirit of rqt_plot / rqt_joint_trajectory_controller): every joint over the program,
with joint limits, step boundaries, cutting spans, and a cursor that follows the simulation. Click to seek.

Modes: "position" (deg) and "step" (absolute joint change per sample, deg; jumps stand out). Drawn with QPainter,
no plotting library needed.
"""
from __future__ import annotations

import math
from typing import List, Optional

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

COLOURS = ["#e8590c", "#2f9e44", "#1c7ed6", "#ae3ec9", "#f2b705", "#12b886", "#868e96"]


class TrajectoryCanvas(QWidget):
    seek = Signal(int)

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(160)
        self.q: Optional[np.ndarray] = None
        self.lower = self.upper = None
        self.bounds: List[int] = []
        self.feed: Optional[np.ndarray] = None
        self.cursor = 0
        self.mode = "position"
        self.visible = [True] * 7
        self.setMouseTracking(True)
        self._hover = None

    def set_data(self, q, lower=None, upper=None, bounds=(), feed=None):
        self.q = None if q is None or len(q) == 0 else np.degrees(np.asarray(q, dtype=float))
        self.lower = None if lower is None else np.degrees(np.asarray(lower, dtype=float))
        self.upper = None if upper is None else np.degrees(np.asarray(upper, dtype=float))
        self.bounds = list(bounds)
        self.feed = None if feed is None else np.asarray(feed, bool)
        self.cursor = 0
        self.update()

    def set_cursor(self, i: int):
        self.cursor = int(i)
        self.update()

    def _series(self):
        if self.mode == "step":
            d = np.abs(np.diff(self.q, axis=0))
            return np.vstack([np.zeros((1, self.q.shape[1])), d])
        return self.q

    def _frame(self):
        return QRectF(52, 8, max(10, self.width() - 60), max(10, self.height() - 30))

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#1f2329"))
        r = self._frame()
        p.setPen(QPen(QColor("#555b66"), 1))
        p.drawRect(r)
        if self.q is None:
            p.setPen(QColor("#9aa0a6"))
            p.drawText(r, Qt.AlignCenter, "Simulate a program to see its joint trajectory")
            p.end()
            return
        Y = self._series()
        n, dof = Y.shape
        show = [j for j in range(dof) if self.visible[j]]
        vals = Y[:, show] if show else Y
        lo, hi = float(np.min(vals)), float(np.max(vals))
        if self.mode == "position" and self.lower is not None:
            pass   # keep the data range; limits are drawn when they fall inside it
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        pad = 0.05 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        X = lambda i: r.left() + (i / max(n - 1, 1)) * r.width()   # noqa: E731
        Yp = lambda v: r.bottom() - (v - lo) / (hi - lo) * r.height()   # noqa: E731
        # cutting spans
        if self.feed is not None and len(self.feed) == n:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(255, 140, 30, 28))
            i = 0
            while i < n:
                if self.feed[i]:
                    j = i
                    while j < n and self.feed[j]:
                        j += 1
                    p.drawRect(QRectF(X(i), r.top(), max(1.0, X(j - 1) - X(i)), r.height()))
                    i = j
                else:
                    i += 1
        # grid and axis labels
        p.setFont(QFont("Segoe UI", 8))
        for k in range(5):
            v = lo + (hi - lo) * k / 4
            y = Yp(v)
            p.setPen(QPen(QColor("#2e333b"), 1))
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(QColor("#9aa0a6"))
            p.drawText(QRectF(0, y - 8, 48, 16), Qt.AlignRight | Qt.AlignVCenter, "{:.0f}".format(v))
        # step boundaries
        p.setPen(QPen(QColor("#4a505a"), 1, Qt.DashLine))
        for b in self.bounds:
            if 0 < b < n:
                p.drawLine(QPointF(X(b), r.top()), QPointF(X(b), r.bottom()))
        # joint limits (position mode)
        if self.mode == "position" and self.lower is not None:
            for j in show:
                c = QColor(COLOURS[j % len(COLOURS)])
                c.setAlpha(90)
                p.setPen(QPen(c, 1, Qt.DotLine))
                for v in (self.lower[j], self.upper[j]):
                    if lo <= v <= hi:
                        p.drawLine(QPointF(r.left(), Yp(v)), QPointF(r.right(), Yp(v)))
        # series (decimated to about two points per pixel)
        stride = max(1, int(n / max(r.width() * 2, 1)))
        idx = np.arange(0, n, stride)
        for j in show:
            path = QPainterPath()
            path.moveTo(X(idx[0]), Yp(Y[idx[0], j]))
            for i in idx[1:]:
                path.lineTo(X(i), Yp(Y[i, j]))
            p.setPen(QPen(QColor(COLOURS[j % len(COLOURS)]), 1.4))
            p.setBrush(Qt.NoBrush)
            p.drawPath(path)
        # cursor and readout
        c = min(max(self.cursor, 0), n - 1)
        p.setPen(QPen(QColor("#ffffff"), 1))
        p.drawLine(QPointF(X(c), r.top()), QPointF(X(c), r.bottom()))
        txt = "  ".join("J{} {:.1f}".format(j + 1, Y[c, j]) for j in show)
        p.setPen(QColor("#e9ecef"))
        p.drawText(QRectF(r.left() + 4, r.top() + 2, r.width() - 8, 16), Qt.AlignLeft,
                   "sample {}/{}   {}{}".format(c + 1, n, txt, " deg" if self.mode == "position" else " deg/sample"))
        p.setPen(QColor("#9aa0a6"))
        p.drawText(QRectF(r.left(), r.bottom() + 4, r.width(), 16), Qt.AlignCenter,
                   "sample (dashed: program steps, shaded: cutting)")
        p.end()

    def mousePressEvent(self, e):
        if self.q is None:
            return
        r = self._frame()
        t = (e.position().x() - r.left()) / max(r.width(), 1)
        self.seek.emit(int(round(min(max(t, 0.0), 1.0) * (len(self.q) - 1))))

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.LeftButton:
            self.mousePressEvent(e)


class TrajectoryPlot(QWidget):
    """The canvas with a mode selector and per-joint toggles."""
    seek = Signal(int)

    def __init__(self):
        super().__init__()
        v = QVBoxLayout(self)
        v.setContentsMargins(2, 2, 2, 2)
        h = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItems(["position", "step"])
        self.mode.currentTextChanged.connect(self._mode)
        h.addWidget(QLabel("Joints"))
        h.addWidget(self.mode)
        self.toggles = []
        from PySide6.QtWidgets import QCheckBox
        for j in range(6):
            cb = QCheckBox("J{}".format(j + 1))
            cb.setChecked(True)
            cb.setStyleSheet("color: {}".format(COLOURS[j]))
            cb.toggled.connect(lambda on, k=j: self._toggle(k, on))
            h.addWidget(cb)
            self.toggles.append(cb)
        self.stats = QLabel()
        h.addWidget(self.stats, 1)
        v.addLayout(h)
        self.canvas = TrajectoryCanvas()
        self.canvas.seek.connect(self.seek)
        v.addWidget(self.canvas, 1)

    def _mode(self, m):
        self.canvas.mode = m
        self.canvas.update()

    def _toggle(self, k, on):
        self.canvas.visible[k] = on
        self.canvas.update()

    def set_data(self, q, lower=None, upper=None, bounds=(), feed=None):
        self.canvas.set_data(q, lower, upper, bounds, feed)
        if q is not None and len(q) > 1:
            d = np.abs(np.diff(np.asarray(q, dtype=float), axis=0))
            worst = int(np.argmax(d.max(axis=1)))
            self.stats.setText("largest step {:.1f} deg at sample {} (J{})".format(
                math.degrees(d.max()), worst + 2, int(np.argmax(d[worst])) + 1))
        else:
            self.stats.setText("")

    def set_cursor(self, i):
        self.canvas.set_cursor(i)
