# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Cutting data for stone and wood: sourced starting ranges keyed by (material, tool kind), and the suggestions
lithocam derives from them. Every entry names its source, the source type and how it was read; lithocam never adds
a number without one. Ranges, not settings: test on the actual stone, machine and tool, and record what works as a
"user test" entry.

Reading notes and the full tables: docs/FEEDS_SPEEDS.md. Confidence "page" = the page was read; "snippet" = only a
search excerpt was seen (check before relying on it).

Suggestions (`suggest`) take the conservative end of each range: the lower surface speed or rpm and the lower feed
or chip load. Wood feed = rpm x flutes x chip load, reduced by 25% at 2 x D depth and 50% at 3 x D (Amana / Freud /
Onsrud guidance). Robots are much less stiff than machine tools, but no source gives a numeric derating; the robot
factor is user-set (1.0 = none) and the suggestion says so.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

STONE = ["granite", "marble", "limestone", "sandstone", "travertine", "quartzite", "quartz_engineered"]
WOOD = ["hardwood", "softwood", "plywood", "mdf", "particleboard", "hpl"]
MATERIALS = STONE + WOOD

SANG = "https://sangtools.com/blog/regular-stone-material-saw-blade-cutting-speed_b12"
DST_BLADE = "https://dynamicstonetools.com/blogs/news/rpm-feed-rate-water-flow-what-the-blade-selector-cant-tell-you"
DST_ROUTER = "https://dynamicstonetools.com/blogs/news/router-bit-speeds-and-feed-rates-for-stone-edge-profiling"
KONSTANTY = "https://link.springer.com/article/10.1007/s00170-021-07577-3"
WEHA = ("https://easternmarble.com/shop/stone-fabrication-supplies/finger-bits/"
        "weha-top-racer-high-speed-cnc-diamond-finger-bit/")
MK = "https://www.mkdiamond.com/coring/tec_speed.html"
SAGE = "https://sage.cnpereading.com/doi/10.1177/17515831261437884"
DST_WJ = "https://dynamicstonetools.com/blogs/news/waterjet-cutting-for-stone-how-it-works-when-to-use-it"
RENNIE = "https://www.rennietool.co.uk/blogs/news/solid-carbide-router-bits-speeds-and-feeds"
EEDGE = "https://engineersedge.com/manufacturing/hardwood_speeds_and_feeds_table_15169.htm"
KRILEK = "Krilek 2014, BioResources 9(1) 1417 (see docs/FEEDS_SPEEDS.md)"
FORSTNER = "https://answers.toolstoday.com/6506422/What-speed-should-you-run-a-Forstner-bit"
SLABWISE = "https://slabwise.com/guides/what-diamond-polishing-pads-are-best-for-granite-finishing"


@dataclass(frozen=True)
class Entry:
    material: str          # one of MATERIALS
    tool: str              # tool kind name (lithocam tool_kind_name)
    quantity: str          # surface_speed | rpm | feed | plunge_feed | chip_load | depth | water | wire_speed | cut_rate | tension
    lo: float
    hi: float
    unit: str
    source: str
    source_type: str       # vendor | vendor blog | dealer | paper | web table | web guide | user test
    confidence: str        # page | snippet
    size: str = ""         # tool size the value applies to ("" = any)
    note: str = ""


def _e(materials, tool, q, lo, hi, unit, src, st, conf, size="", note=""):
    return [Entry(m, tool, q, lo, hi, unit, src, st, conf, size, note) for m in materials]


TABLE: List[Entry] = (
    # ---- diamond disc saws
    _e(["granite"], "disc_saw", "surface_speed", 25, 40, "m/s", SANG, "vendor", "page",
       note="hard 25-30, mid-hard 30-35, soft 35-40; conflicts with 45-60 m/s (Dynamic Stone Tools)")
    + _e(["marble"], "disc_saw", "surface_speed", 40, 50, "m/s", SANG, "vendor", "page", note="hard 40-45, soft 45-50")
    + _e(["sandstone"], "disc_saw", "surface_speed", 50, 65, "m/s", SANG, "vendor", "page")
    + _e(["granite"], "disc_saw", "feed", 200, 500, "mm/min", SANG, "vendor", "page", note="at 6 mm depth")
    + _e(["marble"], "disc_saw", "feed", 1000, 2000, "mm/min", SANG, "vendor", "page", note="at 6 mm depth")
    + _e(["quartz_engineered"], "disc_saw", "feed", 250, 400, "mm/min", SANG, "vendor", "page", note="at 6 mm depth")
    + _e(["granite"], "disc_saw", "water", 22.7, 30.3, "l/min", DST_BLADE, "vendor blog", "page")
    + _e(["marble", "limestone"], "disc_saw", "water", 15.1, 22.7, "l/min", DST_BLADE, "vendor blog", "page")
    + _e(["quartzite"], "disc_saw", "water", 26.5, 34.1, "l/min", DST_BLADE, "vendor blog", "page")
    + _e(["quartz_engineered"], "disc_saw", "water", 22.7, 26.5, "l/min", DST_BLADE, "vendor blog", "page")
    # ---- diamond wire
    + _e(["granite"], "diamond_wire", "wire_speed", 24, 30, "m/s", KONSTANTY, "paper", "snippet")
    + _e(["granite"], "diamond_wire", "cut_rate", 0.9, 1.5, "m2/h", KONSTANTY, "paper", "snippet", note="single wire")
    + _e(["granite"], "diamond_wire", "tension", 2000, 2400, "N", KONSTANTY, "paper", "snippet")
    + _e(["granite"], "multi_wire_saw", "cut_rate", 0.6, 1.0, "m2/h", KONSTANTY, "paper", "snippet")
    # ---- diamond finger bits and stone router bits
    + _e(["granite", "marble", "quartz_engineered"], "flat_end_mill", "rpm", 5500, 8500, "rev/min", WEHA, "dealer",
         "page", size="20-22 mm", note="Weha Top Racer 8-segment")
    + _e(["granite", "marble", "quartz_engineered"], "flat_end_mill", "feed", 610, 914, "mm/min", WEHA, "dealer",
         "page", size="20-22 mm", note="converted from in/min")
    + _e(["marble", "travertine"], "flat_end_mill", "rpm", 8000, 12000, "rev/min", DST_ROUTER, "vendor blog", "page",
         note="no diameter given; conflicts with Weha dealer ranges")
    + _e(["granite"], "flat_end_mill", "rpm", 10000, 14000, "rev/min", DST_ROUTER, "vendor blog", "page",
         note="standard granite; hard granite 12000-16000; no diameter given")
    + _e(["quartzite"], "flat_end_mill", "rpm", 12000, 16000, "rev/min", DST_ROUTER, "vendor blog", "page")
    + _e(["limestone", "sandstone"], "flat_end_mill", "rpm", 7000, 10000, "rev/min", DST_ROUTER, "vendor blog", "page")
    + _e(["marble"], "flat_end_mill", "feed", 1500, 3000, "mm/min", SAGE, "paper", "snippet",
         note="vacuum-brazed tool, test range at 6000-12000 rpm, 1 mm depth, 3 l/min water")
    + _e(STONE, "flat_end_mill", "water", 3, 5, "l/min", DST_ROUTER, "vendor blog", "page")
    + _e(STONE, "ball_end_mill", "water", 3, 5, "l/min", DST_ROUTER, "vendor blog", "page",
         note="router-bit guidance, applied to ball tools by lithocam")
    # ---- edge profiling
    + _e(STONE, "profile_wheel", "feed", 400, 1200, "mm/min", DST_ROUTER, "vendor blog", "page",
         note="eased 800-1200, bullnose 500-900, ogee 400-700, cove 450-750, bevel 700-1100")
    + _e(STONE, "edge_profile_bit", "feed", 400, 1200, "mm/min", DST_ROUTER, "vendor blog", "page",
         note="eased 800-1200, bullnose 500-900, ogee 400-700, cove 450-750, bevel 700-1100")
    + _e(STONE, "profile_wheel", "water", 3, 5, "l/min", DST_ROUTER, "vendor blog", "page")
    # ---- core drilling (rpm by diameter; not stone-specific)
    + _e(STONE, "core_drill", "rpm", 1200, 2000, "rev/min", MK, "vendor", "page", size="2 in")
    + _e(STONE, "core_drill", "rpm", 800, 1300, "rev/min", MK, "vendor", "page", size="3 in")
    + _e(STONE, "core_drill", "rpm", 600, 1000, "rev/min", MK, "vendor", "page", size="4 in")
    + _e(STONE, "core_drill", "rpm", 400, 665, "rev/min", MK, "vendor", "page", size="6 in")
    # ---- polishing (basis not stated by the source)
    + _e(["granite"], "polishing_pad", "rpm", 1500, 3500, "rev/min", SLABWISE, "web guide", "snippet", size="4 in",
         note="50-100 grit 1500-2000; 200-400 2000-2500; 800-1500 2500-3000; 3000 and buff 3000-3500")
    # ---- waterjet
    + _e(["marble"], "waterjet", "feed", 152, 508, "mm/min", "KMT cutting-speed chart (see docs/FEEDS_SPEEDS.md)",
         "vendor", "snippet", note="12.7 mm: 330-508; 19 mm: 152-229")
    + _e(["granite"], "waterjet", "feed", 76, 203, "mm/min", DST_WJ, "vendor blog", "page", note="30 mm, full quality")
    # ---- wood: chip load per tooth at 18000 rpm, depth 1 x D (Rennie Tool table, mm/tooth)
    + [Entry(m, k, "chip_load", lo, hi, "mm/tooth", RENNIE, "vendor", "page", size="{:g} mm".format(d),
             note="18000 rpm, depth 1 x D")
       for k in ("flat_end_mill", "ball_end_mill")
       for m, rows in (("hardwood", [(3.175, .076, .127), (6, .220, .270), (6.35, .229, .279), (8, .310, .370),
                                     (10, .400, .477), (12, .460, .513), (12.7, .483, .534)]),
                       ("plywood", [(3.175, .102, .153), (6, .270, .320), (6.35, .279, .330), (8, .360, .420),
                                    (10, .445, .520), (12, .500, .560), (12.7, .533, .584)]),
                       ("mdf", [(3.175, .102, .178), (6, .320, .396), (6.35, .330, .406), (8, .430, .500),
                                (10, .520, .590), (12, .613, .660), (12.7, .635, .686)]),
                       ("particleboard", [(3.175, .102, .178), (6, .320, .396), (6.35, .330, .406), (8, .430, .500),
                                          (10, .520, .590), (12, .613, .660), (12.7, .635, .686)]),
                       ("hpl", [(3.175, .076, .127), (6, .219, .294), (6.35, .229, .305), (8, .311, .381),
                                (10, .410, .490), (12, .546, .595), (12.7, .584, .635)]))
       for d, lo, hi in rows]
    + [Entry(m, k, "rpm", 18000, 18000, "rev/min", RENNIE, "vendor", "page", note="basis of the chip-load table")
       for m in ("hardwood", "softwood", "plywood", "mdf", "particleboard", "hpl")
       for k in ("flat_end_mill", "ball_end_mill")]
    + [Entry(e.material, "ball_end_mill", e.quantity, e.lo, e.hi, e.unit, e.source, e.source_type, e.confidence,
             e.size, "router-bit guidance, applied to ball tools by lithocam; " + e.note)
       for e in _e(["marble", "travertine"], "x", "rpm", 8000, 12000, "rev/min", DST_ROUTER, "vendor blog", "page")
       + _e(["granite"], "x", "rpm", 10000, 14000, "rev/min", DST_ROUTER, "vendor blog", "page")
       + _e(["limestone", "sandstone"], "x", "rpm", 7000, 10000, "rev/min", DST_ROUTER, "vendor blog", "page")]
    + _e(["hardwood"], "tapered_ball_mill", "chip_load", 0.076, 0.127, "mm/tooth", EEDGE, "web table", "page",
         size="1/8 in", note="2 flutes, 18000 rpm")
    + _e(["hardwood"], "engraver", "chip_load", 0.102, 0.152, "mm/tooth", EEDGE, "web table", "page",
         size="1 in 60 deg", note="2 flutes, 18000 rpm")
    + _e(["softwood"], "disc_saw", "surface_speed", 60, 100, "m/s", KRILEK, "paper", "snippet")
    + _e(["hardwood"], "disc_saw", "surface_speed", 50, 85, "m/s", KRILEK, "paper", "snippet")
    + _e(WOOD, "flat_bottom_drill", "rpm", 1000, 1500, "rev/min", FORSTNER, "vendor", "snippet", size="up to 1 in",
         note="larger bits 500-1000")
)


def lookup(material: str, tool_kind: str, quantity: Optional[str] = None) -> List[Entry]:
    return [e for e in TABLE if e.material == material and e.tool == tool_kind and
            (quantity is None or e.quantity == quantity)]


def rpm_from_surface_speed(v_mps: float, diameter_mm: float) -> float:
    return v_mps * 60000.0 / (math.pi * diameter_mm)


def surface_speed(rpm: float, diameter_mm: float) -> float:
    return rpm * math.pi * diameter_mm / 60000.0


def chip_load_feed(rpm: float, flutes: int, chip_mm: float) -> float:
    return rpm * max(flutes, 1) * chip_mm


def depth_factor(depth_over_diameter: float) -> float:
    """Chip-load reduction for deep cuts (Amana / Freud / Onsrud): 1 x D: 1.0; 2 x D: 0.75; 3 x D and beyond: 0.5."""
    if depth_over_diameter <= 1.0:
        return 1.0
    if depth_over_diameter <= 2.0:
        return 1.0 - 0.25 * (depth_over_diameter - 1.0)
    return max(0.5, 0.75 - 0.25 * (depth_over_diameter - 2.0))


def _size_mm(size: str) -> Optional[float]:
    s = size.split()[0] if size else ""
    try:
        if "-" in s:
            a, b = s.split("-")
            return (float(a) + float(b)) / 2
        v = float(s)
        return v * 25.4 if size.endswith("in") else v
    except ValueError:
        return None


def _fits(e: Entry, d_mm: float) -> bool:
    """An entry applies to any size, or to tools within 25% of its size (ranges: inside the range)."""
    if not e.size:
        return True
    s = e.size.split()[0]
    if "-" in s and not e.size.endswith("in"):
        a, b = (float(v) for v in s.split("-"))
        return a <= d_mm <= b
    m = _size_mm(e.size)
    return m is not None and abs(d_mm - m) <= 0.25 * m


@dataclass
class Suggestion:
    rpm: float = 0.0
    feed: float = 0.0               # mm/min
    water: float = 0.0              # l/min
    sources: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def text(self) -> str:
        parts = []
        if self.rpm:
            parts.append("{:.0f} rev/min".format(self.rpm))
        if self.feed:
            parts.append("{:.0f} mm/min".format(self.feed))
        if self.water:
            parts.append("water {:.1f} l/min".format(self.water))
        return ", ".join(parts) if parts else "no sourced data"


def suggest(tool, material: str, depth_mm: float = 0.0, max_rpm: float = 0.0,
            robot_factor: float = 1.0) -> Suggestion:
    """Conservative starting point for `tool` (a lithocam Tool) in `material`; 0 where no source covers it."""
    from .studio.core import lc
    kind = lc.tool_kind_name(tool.kind)
    D = float(tool.diameter or 0.0)
    out = Suggestion()

    def pick(q):
        es = [e for e in lookup(material, kind, q) if _fits(e, D)]
        # the nearest sized entry first, then unsized ones; page-read before snippet
        es.sort(key=lambda e: (e.size == "", abs((_size_mm(e.size) or D) - D), e.confidence != "page"))
        return es[0] if es else None

    e = pick("rpm")
    if e:
        out.rpm = e.lo
        out.sources.append(e.source)
        if e.note:
            out.notes.append(e.note)
    elif D > 0:
        e = pick("surface_speed")
        if e:
            out.rpm = rpm_from_surface_speed(e.lo, D)
            out.sources.append(e.source)
            out.notes.append("rpm from {:g} m/s at D {:g} mm".format(e.lo, D))
    if out.rpm and max_rpm and out.rpm > max_rpm:
        out.notes.append("capped at the spindle's {:.0f} rev/min".format(max_rpm))
        out.rpm = max_rpm
    e = pick("feed")
    if e:
        out.feed = e.lo
        out.sources.append(e.source)
    else:
        e = pick("chip_load")
        if e and out.rpm:
            f = depth_factor(depth_mm / D) if D and depth_mm else 1.0
            out.feed = chip_load_feed(out.rpm, int(tool.flutes or 2), e.lo) * f
            out.sources.append(e.source)
            out.notes.append("feed = rpm x {} flutes x {:g} mm/tooth{}".format(
                int(tool.flutes or 2), e.lo, "" if f == 1.0 else " x {:.2f} (depth)".format(f)))
    e = pick("water")
    if e:
        out.water = e.lo
        out.sources.append(e.source)
    if robot_factor != 1.0:
        out.feed *= robot_factor
        out.notes.append("robot factor {:.2f} (user-set; no published derating)".format(robot_factor))
    elif out.feed:
        out.notes.append("robots are less stiff than machine tools; no published derating, set one after a test")
    out.sources = list(dict.fromkeys(out.sources))
    return out


def tool_kind_label(tool) -> str:
    """Kind name of a tool: a str, an object with `kind_name`, or a lithocam Tool (via the native module)."""
    if isinstance(tool, str):
        return tool
    if isinstance(getattr(tool, "kind", None), str):
        return tool.kind
    if getattr(tool, "kind_name", None):
        return tool.kind_name
    from .studio.core import lc
    return lc.tool_kind_name(tool.kind)


def pick(material: str, tool_kind: str, quantity: str, diameter_mm: float = 0.0) -> Optional[Entry]:
    """The entry `suggest` would use: nearest sized entry first, then unsized; page-read before snippet. None if
    no source covers it. diameter_mm 0 matches any size."""
    es = [e for e in lookup(material, tool_kind, quantity) if diameter_mm <= 0 or _fits(e, diameter_mm)]
    es.sort(key=lambda e: (e.size == "", abs((_size_mm(e.size) or diameter_mm) - diameter_mm), e.confidence != "page"))
    return es[0] if es else None
