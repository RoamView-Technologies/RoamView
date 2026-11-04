"""Open every RoamView COTS/envelope part and report its extents and frames (local ROS mm) for review."""
import glob, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, MM, ROOT, Part, app, cast

sw = app()
for path in sorted(glob.glob(os.path.join(ROOT, *(sys.argv[1:] or ["COTS/*/RV-*.SLDPRT"])))):
    sw.OpenDoc6(path, K.swDocPART, K.swOpenDocOptions_Silent, "", 0, 0)
    p = Part(sw, sw.ActiveDoc)
    p.rebuild()
    b = cast(p.doc, "IPartDoc").GetPartBox(True)
    lo = (b[2] / MM, b[0] / MM, b[1] / MM); hi = (b[5] / MM, b[3] / MM, b[4] / MM)
    frames = [f.Name for f in p.features() if f.GetTypeName2() == "CoordSys"]
    fr = []
    for n in frames:
        m = p.cs_transform(n)
        o = (m[11] / MM, m[9] / MM, m[10] / MM)
        fr.append(f"{n}@({o[0]:.1f},{o[1]:.1f},{o[2]:.1f})")
    cpm = p.ext.CustomPropertyManager("")
    errs = p.errors()
    print(f"{os.path.basename(path)[:-7]:38s} x[{lo[0]:7.1f},{hi[0]:7.1f}] y[{lo[1]:7.1f},{hi[1]:7.1f}] "
          f"z[{lo[2]:7.1f},{hi[2]:7.1f}]  {' '.join(fr)}  errors={errs}")
    sw.CloseDoc(p.doc.GetTitle())
