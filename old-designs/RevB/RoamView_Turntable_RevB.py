"""
RoamView_Turntable_RevB.py  -  ENGR 490 Capstone, RoamView chassis Rev B

Renders a turntable of RoamView_Chassis_RevB.SLDASM: the view orbits 360 degrees about the
robot's vertical axis while SOLIDWORKS writes one PNG per frame into anim_frames\.
Encode them afterwards, e.g.
    ffmpeg -y -framerate 30 -i anim_frames/frame_%04d.png -c:v libx264 -crf 18 out.mp4

    python RoamView_Turntable_RevB.py [frames] [--test] [--order rt|tr] [--scale 0.82]

Needs Python 3 with pywin32; SOLIDWORKS may be open or closed.
"""
import math
import os
import sys
import time
import winreg

import pythoncom
import win32com.client as wc
from win32com.client import VARIANT, gencache


def sw_folder():
    try:
        k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\SolidWorks")
        names = [winreg.EnumKey(k, i) for i in range(winreg.QueryInfoKey(k)[0])]
        for n in sorted((n for n in names if n.upper().startswith("SOLIDWORKS 20")), reverse=True):
            try:
                s = winreg.OpenKey(k, n + r"\Setup")
                p = winreg.QueryValueEx(s, "SolidWorks Folder")[0]
                if os.path.exists(os.path.join(p, "sldworks.tlb")):
                    return p
            except OSError:
                pass
    except OSError:
        pass
    return r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS"


SWDIR = sw_folder()
_la = pythoncom.LoadTypeLib(os.path.join(SWDIR, "sldworks.tlb")).GetLibAttr()
SW = gencache.EnsureModule(str(_la[0]), _la[1], _la[3], _la[4])
C = {}
_t = pythoncom.LoadTypeLib(os.path.join(SWDIR, "swconst.tlb"))
for _i in range(_t.GetTypeInfoCount()):
    _ti = _t.GetTypeInfo(_i)
    _ta = _ti.GetTypeAttr()
    if _ta.typekind == pythoncom.TKIND_ENUM:
        for _j in range(_ta.cVars):
            _vd = _ti.GetVarDesc(_j)
            C[_ti.GetNames(_vd.memid)[0]] = _vd.value


def cast(o, iface):
    if o is None:
        return None
    return getattr(SW, iface)(o._oleobj_ if hasattr(o, "_oleobj_") else o)


def dbl(v):
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [float(x) for x in v])


HERE = os.path.dirname(os.path.abspath(__file__))
ASM = os.path.join(HERE, "RoamView_Chassis_RevB.SLDASM")


def get_model(sw):
    for d in (sw.GetDocuments() or []):
        md = cast(d, "IModelDoc2")
        if (md.GetPathName() or "").lower() == ASM.lower():
            print("using the open assembly:", md.GetTitle(), flush=True)
            sw.ActivateDoc3(md.GetTitle(), False, C["swDontRebuildActiveDoc"], 0)
            return md
    print("opening", os.path.basename(ASM), flush=True)
    md, errs, warns = sw.OpenDoc6(ASM, C["swDocASSEMBLY"], C["swOpenDocOptions_Silent"], "", 0, 0)
    if md is None:
        raise RuntimeError("could not open the assembly (errors %s, warnings %s)" % (errs, warns))
    return cast(md, "IModelDoc2")


def main(frames=240, test=False, order="rt", scale=0.9):
    t0 = time.time()
    sw = SW.ISldWorks(wc.Dispatch("SldWorks.Application")._oleobj_)
    sw.Visible = True
    model = get_model(sw)
    model.ClearSelection2(True)

    out = os.path.join(HERE, "anim_test" if test else "anim_frames")
    os.makedirs(out, exist_ok=True)
    ext = model.Extension
    opts = C["swSaveAsOptions_Silent"] + C["swSaveAsOptions_Copy"]

    view = cast(model.ActiveView, "IModelView")
    view.EnableGraphicsUpdate = True
    # Frame the shot once, at the angle where the robot is widest on screen, so the
    # framing holds for the whole turn and nothing drifts or clips.
    model.ShowNamedView2("*Isometric", 7)
    view.RotateAboutAxis(math.radians(45.0), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    model.ViewZoomtofit2()
    view.ZoomByFactor(scale)
    view.RotateAboutAxis(math.radians(-45.0), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    model.GraphicsRedraw2()
    mu = cast(sw.GetMathUtility(), "IMathUtility")

    n = 4 if test else frames
    step = 2.0 * math.pi / n                       # one turn, spread over the frames
    for i in range(n):
        if i:                                      # orbit about +Y through the assembly origin
            view.RotateAboutAxis(step, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
            model.GraphicsRedraw2()
        path = os.path.join(out, "frame_%04d.png" % i)
        ok, err, warn = ext.SaveAs(path, 0, opts, None, 0, 0)
        if not (ok and err == 0):
            raise RuntimeError("SaveAs failed on frame %d (error %s)" % (i, err))
        if test or i % 30 == 0:
            print("  frame %3d/%d  %5.1f deg  %.1fs" % (i + 1, n, math.degrees(step * i), time.time() - t0), flush=True)

    model.ShowNamedView2("*Isometric", 7)
    model.ViewZoomtofit2()
    print("DONE %d frames in %.0f s -> %s" % (n, time.time() - t0, out), flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    nf = int(a[0]) if a and a[0].isdigit() else 240
    kw = dict(frames=nf, test="--test" in a, order="tr" if "--order" in a and a[a.index("--order") + 1] == "tr" else "rt")
    if "--scale" in a:
        kw["scale"] = float(a[a.index("--scale") + 1])
    main(**kw)
