"""Build RV-001_ReferenceDock.SLDPRT: a simple stand-in for the charging dock, used for docking checks.

Its origin is the robot's base_footprint when docked, with the same ROS axes, so the top-level assembly
places it with one frame-to-frame mate. Tag face, tag height and contact positions come from the globals,
so the 0.25 m close-range tag check and the pad-height check update with the robot.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, ROOT, Part, Sketch, app, read_globals  # noqa: E402

OUT = os.path.join(ROOT, "01_Layout", "RV-001_ReferenceDock.SLDPRT")
G = read_globals()
Q = lambda n: f'"{n}"'          # noqa: E731
TAGX = G["LENS_X"] + G["DOCK_TAG_DIST"]
TAGX_E = f'{Q("LENS_X")} + {Q("DOCK_TAG_DIST")}'
HALF_W, WALL_T, WALL_H, BASE_T = 150, 10, 500, 5      # dock-only constants (not robot design inputs)


def sized_rect(s, a, b, ax1, ax2, len1, len2, anchor_pt, anchor_exprs):
    """Rectangle a-b, edges along ax1/ax2 set to len1/len2, located by one corner from the origin."""
    r = s.rect(a, b)
    for ax, other, ln in ((ax1, ax2, len1), (ax2, ax1, len2)):
        i, j = "xyz".index(ax), "xyz".index(other)
        m = list(a)
        m[i] = (a[i] + b[i]) / 2
        at = list(m)
        at[j] -= 15
        s.dlen(s.edge(r, tuple(m)), tuple(at), expr=ln)
    c = s.corner(r, anchor_pt)
    for ax, e in anchor_exprs.items():
        at = list(anchor_pt)
        at["xyz".index(ax)] /= 2
        s.d_along(ax, "origin", c, tuple(at), expr=e)
    return r


def build(sw, p):
    p.link_globals()
    pl = p.std_planes()
    tag_face = p.offset_plane("PL_TagFace", "front", TAGX, TAGX_E)
    base_top = p.offset_plane("PL_DockBaseTop", "top", BASE_T, str(BASE_T))

    x0 = G["CONTACT_X"] - 40
    with Sketch(p, pl["top"], "SK_DockBase") as s:
        a, b = (x0, -HALF_W, 0), (TAGX + WALL_T, HALF_W, 0)
        sized_rect(s, a, b, "x", "y", f'{TAGX_E} + {WALL_T} - ({Q("CONTACT_X")} - 40)', str(2 * HALF_W),
                   (x0, HALF_W, 0), {"x": f'{Q("CONTACT_X")} - 40', "y": str(HALF_W)})
    p.extrude(s.feat, "EX_DockBase", BASE_T, str(BASE_T), "z", +1, 0)

    with Sketch(p, pl["top"], "SK_DockWall") as s:
        a, b = (TAGX, -HALF_W, 0), (TAGX + WALL_T, HALF_W, 0)
        sized_rect(s, a, b, "x", "y", str(WALL_T), str(2 * HALF_W), (TAGX, HALF_W, 0),
                   {"x": TAGX_E, "y": str(HALF_W)})
    p.extrude(s.feat, "EX_DockWall", WALL_H, str(WALL_H), "z", +1, 0)

    S, TZ = G["DOCK_TAG_SIZE"], G["DOCK_TAG_Z"]
    with Sketch(p, tag_face, "SK_AprilTag", anchor=(TAGX, 0, 0)) as s:
        a, b = (TAGX, -S / 2, TZ - S / 2), (TAGX, S / 2, TZ + S / 2)
        sized_rect(s, a, b, "y", "z", Q("DOCK_TAG_SIZE"), Q("DOCK_TAG_SIZE"), (TAGX, S / 2, TZ - S / 2),
                   {"y": f'{Q("DOCK_TAG_SIZE")} / 2', "z": f'{Q("DOCK_TAG_Z")} - {Q("DOCK_TAG_SIZE")} / 2'})
    p.extrude(s.feat, "EX_AprilTag", 1, "1", "x", -1, TAGX)

    with Sketch(p, base_top, "SK_DockContacts", anchor=(0, 0, BASE_T)) as s:
        for sy in (1, -1):
            cx, cy = G["CONTACT_X"], sy * G["CONTACT_Y"]
            a, b = (cx - 10, cy - sy * 13, BASE_T), (cx + 10, cy + sy * 13, BASE_T)
            sized_rect(s, a, b, "x", "y", "20", "26", (cx - 10, cy - sy * 13, BASE_T),
                       {"x": f'{Q("CONTACT_X")} - 10', "y": f'{Q("CONTACT_Y")} - 13'})
    p.extrude(s.feat, "EX_DockContacts", G["CONTACT_Z"] - BASE_T, f'{Q("CONTACT_Z")} - {BASE_T}', "z", +1, BASE_T)

    p.set_props(PartNo="RV-001", Description="Reference charging dock (docking checks only)",
                Source="REFERENCE", Revision="A")
    p.save(OUT)
    bad = [k for k, v in p.audit.items() if v != K.swFullyConstrained]
    print("sketches (reopened file):", p.audit)
    print("FULLY DEFINED:", "ALL" if not bad else f"NO -> {bad}")
    print("saved", OUT)


def main():
    sw = app()
    p = Part.new(sw)
    try:
        build(sw, p)
    except Exception:
        if not os.environ.get("RV_KEEP"):
            sw.QuitDoc(p.doc.GetTitle())
        raise


if __name__ == "__main__":
    main()
