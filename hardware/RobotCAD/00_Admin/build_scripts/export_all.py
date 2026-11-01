"""Manufacturing exports into 07_Exports.

STEP (AP214): every part + RV-100 + RV-190          -> 07_Exports/STEP
STL (binary, mm, fine): every 3D-printed part          -> 07_Exports/STL
DXF flat patterns (geometry + bend lines): sheet metal -> 07_Exports/DXF
"""
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, ROOT, app, cast  # noqa: E402

EXP = os.path.join(ROOT, "07_Exports")


def open_doc(sw, path):
    typ = K.swDocASSEMBLY if path.upper().endswith("ASM") else K.swDocPART
    res = sw.OpenDoc6(path, typ, K.swOpenDocOptions_Silent, "", 0, 0)
    doc = cast(res[0] if isinstance(res, tuple) else res, "IModelDoc2")
    sw.ActivateDoc3(doc.GetTitle(), False, 0, None)
    return doc


def save_copy(doc, out):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if os.path.exists(out):
        os.remove(out)
    doc.SaveAs3(out, 0, K.swSaveAsOptions_Silent | K.swSaveAsOptions_Copy)
    return os.path.exists(out) and os.path.getsize(out) > 0


def main():
    sw = app()
    sw.SetUserPreferenceIntegerValue(K.swExportStlUnits, 0)                 # mm
    sw.SetUserPreferenceIntegerValue(K.swSTLQuality, K.swSTLQuality_Fine)
    sw.SetUserPreferenceToggle(K.swSTLBinaryFormat, True)
    rows = []
    parts = sorted(glob.glob(os.path.join(ROOT, "0[345]_*", "**", "RV-*.SLDPRT"), recursive=True))
    for path in parts:
        base = os.path.splitext(os.path.basename(path))[0]
        doc = open_doc(sw, path)
        src = os.path.relpath(path, ROOT)
        ok = save_copy(doc, os.path.join(EXP, "STEP", base + ".step"))
        rows.append((base, "STEP", ok))
        if src.startswith("03_"):
            rows.append((base, "STL", save_copy(doc, os.path.join(EXP, "STL", base + ".stl"))))
        if src.startswith(os.path.join("04_SheetMetal_Weldments", "SheetMetal")):
            out = os.path.join(EXP, "DXF", base + "_flat.dxf")
            if os.path.exists(out):
                os.remove(out)
            pd = cast(doc, "IPartDoc")
            r = pd.ExportToDWG2(out, path, K.swExportToDWG_ExportSheetMetal, True, None, False, False, 1 | 4, None)
            rows.append((base, "DXF", bool(r) and os.path.exists(out)))
        sw.CloseDoc(doc.GetTitle())
    for asm in ("RV-100_RoamView", "RV-190_DockingCheck"):
        path = os.path.join(ROOT, "02_Assemblies", asm + ".SLDASM")
        doc = open_doc(sw, path)
        rows.append((asm, "STEP", save_copy(doc, os.path.join(EXP, "STEP", asm + ".step"))))
        sw.CloseDoc(doc.GetTitle())
    for base, kind, ok in rows:
        print(f"{'OK  ' if ok else 'FAIL'} {kind:4s} {base}")
    print(f"\n{sum(1 for r in rows if r[2])}/{len(rows)} exports written to {EXP}")


if __name__ == "__main__":
    main()
