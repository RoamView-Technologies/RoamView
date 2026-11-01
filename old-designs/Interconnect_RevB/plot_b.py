"""Top views of the Rev B layout, one panel per level, with finger zones and pass-throughs."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly
import shapely
from shapely.geometry import box as sbox

import model_b as M
import rules_b as R

LEVELS = [
    ("Under the lower deck (Y < 93)", lambda y0, y1: y1 <= M.BASE_Y + 0.5, "baseplate"),
    ("Lower deck top (99 - 190)", lambda y0, y1: y0 >= M.BASE_TOP - 0.5 and y1 <= 200 and y0 < 190, "baseplate"),
    ("Under the top deck (150 - 219)", lambda y0, y1: y1 <= M.DECK_Y + 0.5 and y1 > 150, "topdeck"),
    ("Top deck (Y > 222)", lambda y0, y1: y0 >= M.DECK_TOP - 0.5, "topdeck"),
]


def draw_poly(ax, g, **kw):
    if g.is_empty:
        return
    geoms = getattr(g, "geoms", [g])
    for gg in geoms:
        xz = list(gg.exterior.coords)
        ax.add_patch(MPoly([(z, x) for x, z in xz], closed=True, **kw))
        for r in gg.interiors:
            ax.add_patch(MPoly([(z, x) for x, z in r.coords], closed=True, fc="white", ec=kw.get("ec", "k"), lw=0.4))


def main(out="layout_b.png"):
    issues = R.check(verbose=False)
    bad = set()
    for _, _, ids in issues:
        bad.update(ids)
    plates = {pl["id"]: pl for pl in M.plates()}
    fig, axes = plt.subplots(1, 4, figsize=(28, 9.5))
    for ax, (title, sel, plate) in zip(axes, LEVELS):
        pl = plates[plate]
        draw_poly(ax, pl["poly"], fc="#eef1f4", ec="#9aa6b1", lw=0.8)
        for h in pl["holes"]["pass_through"]:
            draw_poly(ax, h, fc="#ffd9a8", ec="#c77700", lw=0.8)
        for p in M.PARTS:
            if p.get("hidden"):
                continue
            shown = False
            col = "#d9534f" if p["id"] in bad else "#2a78d6"
            for pr in p["prims"]:
                g, y0, y1 = R.prim_shape(pr)
                if sel(y0, y1):
                    c = pr.get("color") or [0.5, 0.5, 0.5]
                    draw_poly(ax, g, fc=(*c, 0.55), ec=col, lw=0.7)
                    shown = True
            for z in p["zones"]:
                a, b = z["min"], z["max"]
                if sel(a[1], b[1]):
                    draw_poly(ax, sbox(a[0], a[2], b[0], b[2]), fc="none", ec="#008300", lw=0.8, hatch="///")
            if shown:
                gs = [R.prim_shape(pr)[0] for pr in p["prims"] if sel(*R.prim_shape(pr)[1:])]
                u = shapely.union_all(gs)
                c = u.representative_point()
                ax.text(c.y, c.x, p["name"].replace(" (wireless e-stop)", ""), fontsize=6.5, ha="center", va="center",
                        color="#102030", bbox=dict(fc="white", ec="none", alpha=0.6, pad=0.5))
        ax.set_title(title, fontsize=11)
        ax.set_xlim(145, -145)          # robot right (+Z) drawn on the right when looking down with +X up
        ax.set_xlim(-145, 145)
        ax.set_ylim(-180, 185)
        ax.set_aspect("equal")
        ax.set_xlabel("Z (mm, right +)"); ax.set_ylabel("X (mm, forward +)")
        ax.grid(True, lw=0.3, alpha=0.5)
    fig.suptitle(f"RoamView Rev B layout - {len(issues)} rule issues (red outlines). Hatched green = finger zones; orange = pass-throughs",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main(*sys.argv[1:])
