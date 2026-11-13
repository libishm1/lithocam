# Copyright 2026 The lithocam authors. SPDX-License-Identifier: Apache-2.0
"""Write the URDF, SRDF and kinematics-plugin files for the lithocam robot cells.

    python cells/make_cells.py

Robots
- UR10e: kinematics from Universal Robots' published DH parameters (d1 0.1807, a2 -0.6127, a3 -0.57155,
  d4 0.17415, d5 0.11985, d6 0.11655), in the joint-frame layout of the ROS ur_description.
- KUKA KR120 R2500 PRO: joint origins, axes and limits from ROS-Industrial kuka_experimental
  (kuka_kr120_support/urdf/kr120r2500pro_macro.xacro, BSD).

Collision geometry is primitive (cylinders and boxes) sized to enclose each link roughly. It is for reachability
and gross collision checks, not for clearance work near the robot; swap in the vendor meshes for that.

Cells
- ur10e:           the arm on a fixed base
- kr120:           the arm on a fixed base
- kr120_track:     the arm on a 4 m linear track along x (prismatic joint `track`)
- kr120_turntable: the arm on a fixed base, plus a turntable (revolute `turntable`) carrying the part 1.6 m away
"""
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
PI = math.pi


def cyl(r, l, xyz="0 0 0", rpy="0 0 0"):
    return ('<collision><origin xyz="{}" rpy="{}"/><geometry><cylinder radius="{}" length="{}"/></geometry></collision>'
            '<visual><origin xyz="{}" rpy="{}"/><geometry><cylinder radius="{}" length="{}"/></geometry></visual>'
            ).format(xyz, rpy, r, l, xyz, rpy, r, l)


def box(sx, sy, sz, xyz="0 0 0", rpy="0 0 0"):
    return ('<collision><origin xyz="{}" rpy="{}"/><geometry><box size="{} {} {}"/></geometry></collision>'
            '<visual><origin xyz="{}" rpy="{}"/><geometry><box size="{} {} {}"/></geometry></visual>'
            ).format(xyz, rpy, sx, sy, sz, xyz, rpy, sx, sy, sz)


def link(name, geom=""):
    return '  <link name="{}">{}</link>\n'.format(name, geom)


def visual_path(collision_path):
    """cells/meshes/<robot>/<name>.stl (collision) -> cells/meshes/<robot>/visual/<name>.dae (visual)."""
    robot, name = collision_path.split("/")
    return "{}/visual/{}.dae".format(robot, name.rsplit(".", 1)[0])


def mesh(path, xyz="0 0 0", rpy="0 0 0"):
    """Official meshes at the official offset: the visual COLLADA mesh for display, the collision STL for
    calculation (Tesseract uses the collision geometry, as convex hulls, for contact checks)."""
    o = '<origin xyz="{}" rpy="{}"/>'.format(xyz, rpy)
    gv = '<geometry><mesh filename="package://cells/meshes/{}"/></geometry>'.format(visual_path(path))
    gc = '<geometry><mesh filename="package://cells/meshes/{}"/></geometry>'.format(path)
    return "<visual>{}{}</visual><collision>{}{}</collision>".format(o, gv, o, gc)


D90 = math.radians(90)
# UR10e mesh offsets from Universal_Robots_ROS2_Description config/ur10e/visual_parameters.yaml, expressed in this
# file's link frames (our base_link already includes the base_link_inertia yaw of pi, so the base mesh's own yaw of
# pi cancels it).
UR_MESH = {
    "base_link": ("ur10e/base.stl", "0 0 0", "0 0 0"),
    "shoulder_link": ("ur10e/shoulder.stl", "0 0 0", "0 0 {}".format(PI)),
    "upper_arm_link": ("ur10e/upperarm.stl", "0 0 0.1762", "{} 0 {}".format(D90, -D90)),
    "forearm_link": ("ur10e/forearm.stl", "0 0 0.0393", "{} 0 {}".format(D90, -D90)),
    "wrist_1_link": ("ur10e/wrist1.stl", "0 0 -0.135", "{} 0 0".format(D90)),
    "wrist_2_link": ("ur10e/wrist2.stl", "0 0 -0.12", "0 0 0"),
    "wrist_3_link": ("ur10e/wrist3.stl", "0 -0.0005 -0.1168", "{} 0 0".format(D90)),
}
KR_MESH = {n: ("kr120/{}.stl".format(n), "0 0 0", "0 0 0")
           for n in ("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6")}


def joint(name, typ, parent, child, xyz="0 0 0", rpy="0 0 0", axis=None, lo=None, hi=None, vel=None):
    s = '  <joint name="{}" type="{}"><parent link="{}"/><child link="{}"/><origin xyz="{}" rpy="{}"/>'.format(
        name, typ, parent, child, xyz, rpy)
    if axis:
        s += '<axis xyz="{}"/>'.format(axis)
    if lo is not None:
        s += '<limit effort="0" lower="{:.6f}" upper="{:.6f}" velocity="{:.6f}"/>'.format(lo, hi, vel)
    return s + "</joint>\n"


# ---------------------------------------------------------------- UR10e
UR = dict(d1=0.1807, a2=-0.6127, a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655)


def ur10e_body(prefix=""):
    p = prefix
    d1, a2, a3, d4, d5, d6 = (UR[k] for k in ("d1", "a2", "a3", "d4", "d5", "d6"))
    s = ""
    for n in UR_LINKS:
        s += link(p + n, mesh(*UR_MESH[n]))
    s += link(p + "tool0")
    lim = math.radians(360)
    # base_link is turned 180 degrees about z from the DH base, as base_link -> base_link_inertia in the ROS 2
    # ur_description; with it Tesseract's analytic UR solver matches KDL on most poses (see robot.py: every IK
    # solution is checked against forward kinematics anyway)
    s += joint(p + "shoulder_pan_joint", "revolute", p + "base_link", p + "shoulder_link", "0 0 {}".format(d1),
               rpy="0 0 {}".format(PI), axis="0 0 1", lo=-lim, hi=lim, vel=math.radians(120))
    s += joint(p + "shoulder_lift_joint", "revolute", p + "shoulder_link", p + "upper_arm_link",
               rpy="{} 0 0".format(PI / 2), axis="0 0 1", lo=-lim, hi=lim, vel=math.radians(120))
    s += joint(p + "elbow_joint", "revolute", p + "upper_arm_link", p + "forearm_link", "{} 0 0".format(a2),
               axis="0 0 1", lo=-PI, hi=PI, vel=math.radians(180))
    s += joint(p + "wrist_1_joint", "revolute", p + "forearm_link", p + "wrist_1_link", "{} 0 {}".format(a3, d4),
               axis="0 0 1", lo=-lim, hi=lim, vel=math.radians(180))
    s += joint(p + "wrist_2_joint", "revolute", p + "wrist_1_link", p + "wrist_2_link", "0 {} 0".format(-d5),
               rpy="{} 0 0".format(PI / 2), axis="0 0 1", lo=-lim, hi=lim, vel=math.radians(180))
    s += joint(p + "wrist_3_joint", "revolute", p + "wrist_2_link", p + "wrist_3_link", "0 {} 0".format(d6),
               rpy="{} {} {}".format(PI / 2, PI, PI), axis="0 0 1", lo=-lim, hi=lim, vel=math.radians(180))
    s += joint(p + "wrist_3-tool0", "fixed", p + "wrist_3_link", p + "tool0")
    return s


UR_LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link",
            "wrist_3_link"]

# ---------------------------------------------------------------- KR120 R2500 PRO
KR_LIMITS = [(-185, 185, 156), (-155, 35, 156), (-130, 154, 156), (-350, 350, 330), (-130, 130, 330),
             (-350, 350, 615)]


def kr120_body(prefix=""):
    p = prefix
    s = ""
    for n in KR_LINKS:
        s += link(p + n, mesh(*KR_MESH[n]))
    s += link(p + "flange") + link(p + "tool0")
    org = ["0 0 0.675", "0.35 0 0", "1.150 0 0", "1.0 0 -0.041", "0 0 0", "0 0 0"]
    axes = ["0 0 -1", "0 1 0", "0 1 0", "-1 0 0", "0 1 0", "-1 0 0"]
    parents = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5"]
    for i in range(6):
        lo, hi, v = KR_LIMITS[i]
        s += joint(p + "joint_a{}".format(i + 1), "revolute", p + parents[i], p + "link_{}".format(i + 1), org[i],
                   axis=axes[i], lo=math.radians(lo), hi=math.radians(hi), vel=math.radians(v))
    s += joint(p + "joint_a6-flange", "fixed", p + "link_6", p + "flange", "0.215 0 0")
    s += joint(p + "flange-tool0", "fixed", p + "flange", p + "tool0", rpy="0 {} 0".format(PI / 2))
    return s


KR_LINKS = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]

# ---------------------------------------------------------------- KUKA KR 480 R3330 MT and KR 500 R2830 MT
# Dimensions from the KUKA KR 500 FORTEC operating instructions (BA KR 500 FORTEC V2, 16.04.2014, section 4,
# "Working envelope" figures, public PDF at
# http://supportwop.com/IntegrationRobot/content/2-Robots/Charges_lourdes(300-1300kg)/Fortec/BA_KR_500_FORTEC_en.pdf):
#   Fig. 4-2, KR 500 R2830:  A1-A2 offset 500, base to A2 1045, A2 to A3 1300, A3 to A4/A5 point 1025,
#                            A4/A5 point to flange 290, A3-to-wrist offset 55, maximum reach 2826
#   Fig. 4-27, KR 340 R3330: same except A3 to A4/A5 point 1525, maximum reach 3326
# Axis ranges (same tables): A1 +-185, A2 +20/-130, A3 +144/-100, A4 +-350, A5 +-120, A6 +-350 degrees; speeds with
# rated payload A1 90, A2 80, A3 75, A4 90, A5 83, A6 130 deg/s. Maximum reach is measured to the A4/A5
# intersection (the working-envelope reference point), not to the flange.
# The MT (machine tool) variants are not in that manual; the same manual and the KUKA product page state the
# KR 500 R2830 MT has 500 kg and 2826 mm reach and the KR 480 R3330 MT has 480 kg and 3326 mm, the figures of the
# R2830 and R3330 arms, so the arm dimensions above are used for them. The figure gives the 55 mm offset
# without a visible sign; its sign (A4/A5 point 55 mm ABOVE the A3 axis in the L pose, opposite to the KR120) is
# taken from the Q-LAB KR500 link meshes read at run time (tests/test_robomove.py checks it when the clone is
# present). Link shapes are primitive boxes and cylinders (no public meshes).
KR_FORTEC_LIMITS = [(-185, 185, 90), (-130, 20, 80), (-100, 144, 75), (-350, 350, 90), (-120, 120, 83),
                    (-350, 350, 130)]
KR_FORTEC = {
    "kr500_r2830_mt": dict(label="KUKA KR 500 R2830 MT", a1=0.500, a2=0.055, c1=1.045, c2=1.300, c3=1.025,
                           c4=0.290),
    "kr480_r3330_mt": dict(label="KUKA KR 480 R3330 MT", a1=0.500, a2=0.055, c1=1.045, c2=1.300, c3=1.525,
                           c4=0.290),
}


def fortec_body(p_):
    """URDF body of a KR QUANTEC/FORTEC-class arm from the dimensions in KR_FORTEC[...] (joint layout as kr120)."""
    a1, a2, c1, c2, c3, c4 = (p_[k] for k in ("a1", "a2", "c1", "c2", "c3", "c4"))
    X = "0 {} 0".format(PI / 2)            # cylinder axis z -> x
    Y = "{} 0 0".format(PI / 2)            # cylinder axis z -> y
    geom = {
        "base_link": cyl(0.50, 0.40, "0 0 0.20"),
        "link_1": cyl(0.42, c1, "0 0 {}".format(-c1 / 2)) + cyl(0.33, 0.70, "{} 0 0".format(a1), Y),
        "link_2": box(c2, 0.36, 0.40, "{} 0 0".format(c2 / 2)),
        "link_3": box(c3, 0.30, 0.30, "{} 0 {}".format(c3 / 2, a2 / 2)) + cyl(0.26, 0.50, "0 0 0", Y),
        "link_4": cyl(0.16, 0.30, "0 0 0", X),
        "link_5": cyl(0.15, 0.24, "0 0 0", Y),
        "link_6": cyl(0.13, 0.12, "0.06 0 0", X),
    }
    s = ""
    for n in KR_LINKS:
        s += link(n, geom[n])
    s += link("flange") + link("tool0")
    org = ["0 0 {}".format(c1), "{} 0 0".format(a1), "{} 0 0".format(c2), "{} 0 {}".format(c3, a2), "0 0 0", "0 0 0"]
    axes = ["0 0 -1", "0 1 0", "0 1 0", "-1 0 0", "0 1 0", "-1 0 0"]
    parents = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5"]
    for i in range(6):
        lo, hi, v = KR_FORTEC_LIMITS[i]
        s += joint("joint_a{}".format(i + 1), "revolute", parents[i], "link_{}".format(i + 1), org[i], axis=axes[i],
                   lo=math.radians(lo), hi=math.radians(hi), vel=math.radians(v))
    s += joint("joint_a6-flange", "fixed", "link_6", "flange", "{} 0 0".format(c4))
    s += joint("flange-tool0", "fixed", "flange", "tool0", rpy="0 {} 0".format(PI / 2))
    return s


def fortec_plugins(p_):
    """OPW parameters for the same arm (same joint conventions as the KR120)."""
    opw = KR_OPW
    for k, old in (("a1", "0.350"), ("a2", "0.041"), ("c1", "0.675"), ("c2", "1.150"), ("c3", "1.000"),
                   ("c4", "0.215")):
        v = -p_[k] if k == "a2" else p_[k]   # OPW a2 is the negative of the wrist height above the A3 axis
        opw = opw.replace("{}: {}".format(k, old), "{}: {:.3f}".format(k, v))
    return KR_PLUGINS.replace(KR_OPW, opw)



# ---------------------------------------------------------------- files

# ---------------------------------------------------------------- wire-saw tools
# A U-shaped diamond-wire bow saw on tool0 (tool x = wire direction, tool z = out of the flange). The wire runs
# along x at z = depth between the bottoms of two pulleys at x = -half and +half; its return run goes back over
# the beam. Frame, pulleys and drive motor are collision geometry; the cutting wire is not (it is meant to pass
# through the stone).
WIRE_TOOLS = {
    # UR10e (payload 12.5 kg): light aluminium bow, 440 mm wire span, 80 mm pulleys
    "ur10e": dict(half=0.22, depth=0.25, rp=0.04, beam=0.035, plate=(0.10, 0.10, 0.015), motor=(0.07, 0.07, 0.09)),
    # KR120 (payload 120 kg): steel bow, 800 mm span, 180 mm pulleys, larger drive
    "kr120": dict(half=0.40, depth=0.60, rp=0.09, beam=0.07, plate=(0.22, 0.22, 0.03), motor=(0.16, 0.16, 0.20)),
}
# the KR 480 and KR 500 MT cells carry the same steel bow (480 and 500 kg payload)
WIRE_TOOLS["kr480_r3330_mt"] = WIRE_TOOLS["kr500_r2830_mt"] = WIRE_TOOLS["kr120"]


def wire_tool_parts(t):
    """[(name, kind, size, xyz, rpy)] in tool0 coordinates; kind box (size = x y z) or cylinder (radius, length)."""
    h, d, rp, w = t["half"], t["depth"], t["rp"], t["beam"]
    px, py, pz = t["plate"]
    zb = pz + w / 2                                   # beam centre height
    arm_len = (d - rp) - zb
    parts = [
        ("plate", "box", (px, py, pz), (0, 0, pz / 2), (0, 0, 0)),
        ("beam", "box", (2 * h + 2 * rp, w, w), (0, 0, zb), (0, 0, 0)),
        ("arm_a", "box", (w, w, arm_len), (-h, 0, zb + arm_len / 2), (0, 0, 0)),
        ("arm_b", "box", (w, w, arm_len), (h, 0, zb + arm_len / 2), (0, 0, 0)),
        ("pulley_a", "cylinder", (rp, w * 0.8), (-h, 0, d - rp), (PI / 2, 0, 0)),
        ("pulley_b", "cylinder", (rp, w * 0.8), (h, 0, d - rp), (PI / 2, 0, 0)),
        ("motor", "box", t["motor"], (-h * 0.5, 0, zb + w / 2 + t["motor"][2] / 2), (0, 0, 0)),
    ]
    return parts


def wire_tool_xml(robot_name, parent="tool0"):
    s = ""
    t = WIRE_TOOLS[robot_name]
    for name, kind, size, xyz, rpy in wire_tool_parts(t):
        o = '<origin xyz="{} {} {}" rpy="{} {} {}"/>'.format(*xyz, *rpy)
        g = ('<box size="{} {} {}"/>'.format(*size) if kind == "box"
             else '<cylinder radius="{}" length="{}"/>'.format(*size))
        s += link("wiresaw_" + name, "<visual>{0}<geometry>{1}</geometry></visual>"
                                     "<collision>{0}<geometry>{1}</geometry></collision>".format(o, g))
        s += joint("tool0-wiresaw_" + name, "fixed", parent, "wiresaw_" + name)
    return s


WIRE_TOOL_LINKS = ["wiresaw_" + p[0] for p in wire_tool_parts(WIRE_TOOLS["ur10e"])]


def urdf(name, body):
    return ('<?xml version="1.0"?>\n<robot name="{}" xmlns:tesseract="https://github.com/tesseract-robotics/tesseract"'
            ' tesseract:make_convex="true">\n  <link name="world"/>\n{}</robot>\n').format(name, body)


def disable(links, extra=()):
    s = ""
    pairs = set()
    for i in range(len(links)):
        for j in range(i + 1, min(i + 3, len(links))):
            pairs.add((links[i], links[j]))
    pairs |= set(extra)
    for a, b in sorted(pairs):
        s += '  <disable_collisions link1="{}" link2="{}" reason="Adjacent"/>\n'.format(a, b)
    return s


def srdf(name, group_xml, plugins_file, links, extra_disable=()):
    return ('<?xml version="1.0"?>\n<robot name="{}">\n{}'
            '  <kinematics_plugin_config filename="package://cells/{}"/>\n'
            '  <contact_managers_plugin_config filename="package://cells/contact_manager_plugins.yaml"/>\n'
            '{}</robot>\n').format(name, group_xml, plugins_file, disable(links, extra_disable))


UR_PLUGINS = """kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_ur_factories
    - tesseract_kinematics_kdl_factories
  fwd_kin_plugins:
    manipulator:
      default: KDLFwdKinChain
      plugins:
        KDLFwdKinChain:
          class: KDLFwdKinChainFactory
          config: {base_link: base_link, tip_link: tool0}
  inv_kin_plugins:
    manipulator:
      default: URInvKin
      plugins:
        URInvKin:
          class: URInvKinFactory
          config:
            base_link: base_link
            tip_link: tool0
            params: {d1: 0.1807, a2: -0.6127, a3: -0.57155, d4: 0.17415, d5: 0.11985, d6: 0.11655}
        KDLInvKinChainLMA:
          class: KDLInvKinChainLMAFactory
          config: {base_link: base_link, tip_link: tool0}
"""

# OPW parameters for the KR120 R2500 PRO, checked against KDL forward kinematics by tests/test_cells.py.
KR_OPW = """          class: OPWInvKinFactory
          config:
            base_link: base_link
            tip_link: tool0
            params:
              a1: 0.350
              a2: 0.041
              b: 0.000
              c1: 0.675
              c2: 1.150
              c3: 1.000
              c4: 0.215
              offsets: [0, -1.57079632679, 0, 0, 0, 0]
              sign_corrections: [-1, 1, 1, -1, 1, -1]
"""

KR_PLUGINS = """kinematic_plugins:
  search_libraries:
    - tesseract_kinematics_opw_factories
    - tesseract_kinematics_kdl_factories
  fwd_kin_plugins:
    manipulator:
      default: KDLFwdKinChain
      plugins:
        KDLFwdKinChain:
          class: KDLFwdKinChainFactory
          config: {base_link: base_link, tip_link: tool0}
  inv_kin_plugins:
    manipulator:
      default: OPWInvKin
      plugins:
        KDLInvKinChainLMA:
          class: KDLInvKinChainLMAFactory
          config: {base_link: base_link, tip_link: tool0}
        OPWInvKin:
""" + KR_OPW

CONTACT = """contact_manager_plugins:
  search_libraries:
    - tesseract_collision_bullet_factories
    - tesseract_collision_fcl_factories
  discrete_plugins:
    default: BulletDiscreteBVHManager
    plugins:
      BulletDiscreteBVHManager:
        class: BulletDiscreteBVHManagerFactory
  continuous_plugins:
    default: BulletCastBVHManager
    plugins:
      BulletCastBVHManager:
        class: BulletCastBVHManagerFactory
"""


def tool_disable(last_link, wrist_links):
    """The bow is bolted to the last link: no checks between its parts, nor between it and the wrist links it is
    mounted on. Every other robot link, and every obstacle, is checked against it."""
    pairs = set()
    for a in WIRE_TOOL_LINKS:
        for w in wrist_links + [last_link]:
            pairs.add((w, a))
        for b in WIRE_TOOL_LINKS:
            if a < b:
                pairs.add((a, b))
    return pairs


def main():
    (HERE / "contact_manager_plugins.yaml").write_text(CONTACT, encoding="utf-8")
    # UR10e with its wire-saw bow
    (HERE / "ur10e.urdf").write_text(urdf("ur10e", joint("world-base", "fixed", "world", "base_link") +
                                          ur10e_body() + wire_tool_xml("ur10e")), encoding="utf-8")
    (HERE / "ur10e_plugins.yaml").write_text(UR_PLUGINS, encoding="utf-8")
    (HERE / "ur10e.srdf").write_text(srdf("ur10e", '  <group name="manipulator"><chain base_link="base_link" '
                                          'tip_link="tool0"/></group>\n', "ur10e_plugins.yaml", UR_LINKS,
                                          tool_disable("wrist_3_link", ["wrist_2_link"])), encoding="utf-8")
    # KR120 with its wire-saw bow
    (HERE / "kr120.urdf").write_text(urdf("kr120", joint("world-base", "fixed", "world", "base_link") +
                                          kr120_body() + wire_tool_xml("kr120")), encoding="utf-8")
    (HERE / "kr120_plugins.yaml").write_text(KR_PLUGINS, encoding="utf-8")
    (HERE / "kr120.srdf").write_text(srdf("kr120", '  <group name="manipulator"><chain base_link="base_link" '
                                          'tip_link="tool0"/></group>\n', "kr120_plugins.yaml", KR_LINKS,
                                          tool_disable("link_6", ["link_5"])), encoding="utf-8")
    # KR 480 R3330 MT and KR 500 R2830 MT with the same bow
    for name, p_ in KR_FORTEC.items():
        (HERE / (name + ".urdf")).write_text(urdf(name, joint("world-base", "fixed", "world", "base_link") +
                                                  fortec_body(p_) + wire_tool_xml(name)), encoding="utf-8")
        (HERE / (name + "_plugins.yaml")).write_text(fortec_plugins(p_), encoding="utf-8")
        (HERE / (name + ".srdf")).write_text(srdf(name, '  <group name="manipulator"><chain base_link="base_link" '
                                                  'tip_link="tool0"/></group>\n', name + "_plugins.yaml", KR_LINKS,
                                                  tool_disable("link_6", ["link_5"])), encoding="utf-8")
    print("wrote", sorted(p.name for p in HERE.iterdir() if p.suffix in (".urdf", ".srdf", ".yaml")))


if __name__ == "__main__":
    main()
