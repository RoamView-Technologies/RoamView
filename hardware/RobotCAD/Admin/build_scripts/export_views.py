"""Save shaded review images of a RoamView document: python export_views.py <doc relative to RobotCAD> <out dir> [views...]
Views are SolidWorks named views (*Isometric, *Front, *Left, *Top, *Back, *Trimetric, *Dimetric)."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, ROOT, app, cast  # noqa: E402

doc_rel, out = sys.argv[1], os.path.normpath(sys.argv[2])
views = sys.argv[3:] or ["*Isometric", "*Front", "*Left", "*Top", "*Back"]
os.makedirs(out, exist_ok=True)
sw = app()
path = os.path.normpath(os.path.join(ROOT, doc_rel))
typ = K.swDocASSEMBLY if path.upper().endswith("ASM") else K.swDocPART
res = sw.OpenDoc6(path, typ, K.swOpenDocOptions_Silent, "", 0, 0)
doc = cast(res[0] if isinstance(res, tuple) else res, "IModelDoc2")
for k, v in (("swDisplayPlanes", False), ("swDisplaySketches", False), ("swDisplayCoordSystems", False),
             ("swDisplayAxes", False), ("swDisplayOrigins", False), ("swDisplayPoints", False)):
    if hasattr(K, k):
        doc.Extension.SetUserPreferenceToggle(getattr(K, k), 0, v)
sw.ActivateDoc3(doc.GetTitle(), False, 0, None)
if typ == K.swDocASSEMBLY:                       # hide the skeleton (its sketch rays swamp zoom-to-fit)
    for comp in cast(doc, "IAssemblyDoc").GetComponents(False) or []:
        comp = cast(comp, "IComponent2")
        if os.path.basename(comp.GetPathName()).startswith("RV-000"):
            comp.Visible = K.swComponentHidden
doc.ViewDisplayShaded()
base = os.path.splitext(os.path.basename(path))[0]
for v in views:
    doc.ShowNamedView2(v, -1)
    doc.ViewZoomtofit2()
    doc.GraphicsRedraw2()
    f = os.path.join(out, f"{base}_{v.strip('*').lower()}.png")
    doc.SaveAs3(f, 0, K.swSaveAsOptions_Silent | K.swSaveAsOptions_Copy)
    print(f, os.path.exists(f))
sw.QuitDoc(doc.GetTitle())                       # discard the temporary hide
