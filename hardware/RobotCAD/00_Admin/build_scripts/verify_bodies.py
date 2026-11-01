"""List the solid bodies of RoamView parts (ROS mm boxes) and flag overlapping body boxes."""
import glob, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, MM, ROOT, Part, app, cast

def boxes(p):
    out = []
    for b in cast(p.doc, "IPartDoc").GetBodies2(0, False) or []:
        b = cast(b, "IBody2"); bb = b.GetBodyBox()
        lo = (bb[2] / MM, bb[0] / MM, bb[1] / MM); hi = (bb[5] / MM, bb[3] / MM, bb[4] / MM)
        out.append((b.Name, lo, hi))
    return out

def overlap(a, b, tol=0.05):
    return all(min(a[2][i], b[2][i]) - max(a[1][i], b[1][i]) > tol for i in range(3))

sw = app()
for pat in sys.argv[1:]:
    for path in sorted(glob.glob(os.path.join(ROOT, pat))):
        sw.OpenDoc6(path, K.swDocPART, K.swOpenDocOptions_Silent, "", 0, 0)
        p = Part(sw, sw.ActiveDoc); p.rebuild()
        bx = boxes(p)
        types = {f.GetTypeName2() for f in p.features()}
        print(f"\n{os.path.basename(path)}  bodies={len(bx)}  weldment={'WeldMemberFeat' in types or 'Weldment' in types or 'WeldmentFeature' in types}  errors={p.errors()}")
        for n, lo, hi in bx:
            print(f"   {n:28s} x[{lo[0]:7.1f},{hi[0]:7.1f}] y[{lo[1]:7.1f},{hi[1]:7.1f}] z[{lo[2]:6.1f},{hi[2]:6.1f}]")
        clash = [(a[0], b[0]) for i, a in enumerate(bx) for b in bx[i + 1:] if overlap(a, b)]
        print("   overlapping body boxes:", clash or "none")
        sw.CloseDoc(p.doc.GetTitle())
