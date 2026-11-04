"""Production drawings (Drawings, PDF copies in Drawings/PDF).

Made parts (PRINT / SHEET / WELD): A3, third-angle Front/Top/Right + isometric, model sketch dimensions imported.
Sheet metal adds the flat pattern; weldments add the cut list. RV-100 gets an assembly drawing with a
parts-only BOM table and balloons.

    python drawings.py            # all made parts + RV-100
    python drawings.py RV-434    # one part
"""
import glob
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, MM, ROOT, app, cast  # noqa: E402

OUT = os.path.join(ROOT, "Drawings")
LANG = r"C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\lang\english"
SCALES = [(2, 1), (1, 1), (1, 2), (1, 5), (1, 10), (1, 20)]       # (numerator, denominator), largest first
FORMAT = r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2026\lang\english\sheetformat3 - iso.slddrt"
W, H = 0.42, 0.297
TB_TOP = 0.062                       # ISO A3 title block height (bottom right)


def template(sw):
    t = sw.GetUserPreferenceStringValue(K.swDefaultTemplateDrawing)
    return t if t and os.path.exists(t) else r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2026\templates\Drawing.DRWDOT"


def part_size_mm(sw, path):
    res = sw.OpenDoc6(path, K.swDocPART if path.upper().endswith("PRT") else K.swDocASSEMBLY,
                      K.swOpenDocOptions_Silent, "", 0, 0)
    doc = cast(res[0] if isinstance(res, tuple) else res, "IModelDoc2")
    if path.upper().endswith("PRT"):
        b = cast(doc, "IPartDoc").GetPartBox(True)
    else:
        b = cast(doc, "IAssemblyDoc").GetBox(0)
    return max(abs(b[3] - b[0]), abs(b[4] - b[1]), abs(b[5] - b[2])) / MM


def new_sheet(sw, scale, a2=False):
    paper, fmt, w, h = ((K.swDwgPaperA2size, FORMAT.replace("a3 - iso", "a2 - iso"), 0.594, 0.42) if a2
                        else (K.swDwgPaperA3size, FORMAT, W, H))
    doc = cast(sw.NewDocument(template(sw), paper, w, h), "IModelDoc2")
    dwg = cast(doc, "IDrawingDoc")
    sheet = cast(dwg.GetCurrentSheet(), "ISheet")
    ok = dwg.SetupSheet6(sheet.GetName(), paper, K.swDwgTemplateCustom, scale[0], scale[1], False, fmt,
                         w, h, "Default", True, 0, 0, 0, 0, 0, 0)
    if not ok:
        sheet.SetProperties2(paper, K.swDwgTemplateA2size if a2 else K.swDwgTemplateA3size, scale[0], scale[1],
                             False, w, h, True)
        sheet.SetTemplateName(fmt)
        sheet.ReloadTemplate(True)
    got = sheet.GetProperties2()
    if abs(got[5] - w) > 1e-3:
        raise RuntimeError(f"sheet size {got[5]:.3f} x {got[6]:.3f}, wanted {w} x {h}")
    return doc, dwg


def set_scale(dwg, scale):
    sheet = cast(dwg.GetCurrentSheet(), "ISheet")
    sheet.SetScale(scale[0], scale[1], True, False)


def views(dwg):
    v = cast(cast(dwg.GetFirstView(), "IView").GetNextView(), "IView")
    out = []
    while v is not None:
        out.append(v)
        v = cast(v.GetNextView(), "IView")
    return out


def size(v):
    o = v.GetOutline()
    return o[2] - o[0], o[3] - o[1]


def move(v, x, y):
    import pythoncom
    import win32com.client as w32
    v.Position = w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [x, y])


def layout(dwg, has_flat):
    """Third-angle layout: front bottom-left, top above, right beside; iso top-right; flat bottom-right above
    the title block. Returns True if nothing overlaps and everything is on the sheet."""
    vs = views(dwg)
    front, top, right, iso = vs[0], vs[1], vs[2], vs[3]
    flat = vs[4] if has_flat and len(vs) > 4 else None
    g = 0.022
    wf, hf = size(front); wt, ht = size(top); wr, hr = size(right); wi, hi = size(iso)
    x0, y0 = 0.03, 0.035
    fx, fy = x0 + wf / 2, y0 + max(hf, hr) / 2
    move(front, fx, fy); move(top, fx, fy + hf / 2 + g + ht / 2); move(right, fx + wf / 2 + g + wr / 2, fy)
    move(iso, W - 0.02 - wi / 2, H - 0.02 - hi / 2)
    boxes = [v.GetOutline() for v in (front, top, right, iso)]
    if flat is not None:
        wl, hl = size(flat)
        move(flat, W - 0.02 - wl / 2, TB_TOP + 0.012 + hl / 2)
        boxes.append(flat.GetOutline())
    boxes = [v.GetOutline() for v in vs]
    inside = all(b[0] >= 0.01 and b[1] >= 0.01 and b[2] <= W - 0.01 and b[3] <= H - 0.01 for b in boxes)
    clear = all(min(a[2], b[2]) - max(a[0], b[0]) < -0.004 or min(a[3], b[3]) - max(a[1], b[1]) < -0.004
                for i, a in enumerate(boxes) for b in boxes[i + 1:])
    tb = all(not (b[2] > W - 0.19 and b[1] < TB_TOP) for b in boxes)
    return inside and clear and tb


def tidy_dims(doc, dwg):
    try:
        doc.ClearSelection2(True)
        for v in views(dwg):
            for a in v.GetAnnotations() or []:
                cast(a, "IAnnotation").Select3(True, None)
        doc.Extension.AlignDimensions(K.swAlignDimensionType_AutoArrange, 0.001)
        doc.ClearSelection2(True)
    except Exception as e:
        print(f"   (dimension arrange skipped: {e})")


def finish(sw, doc, dwg, base, annotate=True):
    try:
        if not annotate:
            raise RuntimeError("weldment: cut list instead of sketch dimensions")
        dwg.InsertModelAnnotations3(K.swImportModelItemsFromEntireModel, K.swInsertDimensionsMarkedForDrawing,
                                    True, False, False, True)
    except Exception as e:
        print(f"   (annotations skipped: {e})")
    tidy_dims(doc, dwg)
    doc.ForceRebuild3(False)
    path = os.path.join(OUT, base + ".SLDDRW")
    if os.path.exists(path):
        os.remove(path)
    doc.SaveAs3(path, 0, K.swSaveAsOptions_Silent)
    pdf = os.path.join(OUT, "PDF", base + ".pdf")
    os.makedirs(os.path.dirname(pdf), exist_ok=True)
    if os.path.exists(pdf):
        os.remove(pdf)
    doc.SaveAs3(pdf, 0, K.swSaveAsOptions_Silent | K.swSaveAsOptions_Copy)
    sw.CloseDoc(doc.GetTitle())
    return os.path.exists(path), os.path.exists(pdf)


def part_drawing(sw, path):
    base = os.path.splitext(os.path.basename(path))[0]
    rel = os.path.relpath(path, ROOT)
    sz = part_size_mm(sw, path)
    k = next((i for i, (n, d) in enumerate(SCALES) if sz * n / d <= 75), len(SCALES) - 1)
    doc, dwg = new_sheet(sw, SCALES[k])
    if not dwg.Create3rdAngleViews2(path):
        raise RuntimeError("Create3rdAngleViews2 failed")
    dwg.CreateDrawViewFromModelView3(path, "*Isometric", 0.34, 0.215, 0)
    sheet_metal = rel.startswith(os.path.join("SheetMetal_Weldments", "SheetMetal"))
    if sheet_metal:
        fp = dwg.CreateFlatPatternViewFromModelView3(path, "", 0.34, 0.085, 0, False, False)
        if fp is None:
            print("   (flat pattern view failed)")
    for v in views(dwg):                                   # every view follows the sheet scale
        v.UseSheetScale = 1
    set_scale(dwg, SCALES[k])
    for _ in range(4):
        doc.ForceRebuild3(False)
        if layout(dwg, sheet_metal) or k == len(SCALES) - 1:
            break
        k += 1
        set_scale(dwg, SCALES[k])
    if rel.startswith(os.path.join("SheetMetal_Weldments", "Weldments")):
        v = cast(dwg.GetFirstView(), "IView")            # sheet
        v = cast(v.GetNextView(), "IView")               # first model view
        t = None
        for cfg in ("Default", "", None):                  # the configuration name the API expects varies
            for tpl in ("weldtable-standard.sldwldtbt", "cut list.sldwldtbt"):
                try:
                    t = v.InsertWeldmentTable(False, 0.03, 0.282, K.swBOMConfigurationAnchor_TopLeft, cfg or "",
                                              os.path.join(LANG, tpl))
                except Exception:
                    t = None
                if t is not None:
                    break
            if t is not None:
                break
        print("   cut list table:", "OK" if t is not None else "failed")
    ok = finish(sw, doc, dwg, base, annotate=not rel.startswith(os.path.join("SheetMetal_Weldments", "Weldments")))
    n, d = SCALES[k]
    return base, f"{n}:{d}", ok


def assembly_drawing(sw):
    path = os.path.join(ROOT, "Assemblies", "RV-100_RoamView.SLDASM")
    part_size_mm(sw, path)
    doc, dwg = new_sheet(sw, (1, 4), a2=True)
    views = [dwg.CreateDrawViewFromModelView3(path, "*Front", 0.09, 0.29, 0),
             dwg.CreateDrawViewFromModelView3(path, "*Left", 0.23, 0.29, 0),
             dwg.CreateDrawViewFromModelView3(path, "*Top", 0.09, 0.12, 0),
             dwg.CreateDrawViewFromModelView3(path, "*Isometric", 0.25, 0.12, 0)]
    for v in views:
        if v is not None:
            cast(v, "IView").UseSheetScale = 1
    for v in views:                                       # hide the master layout skeleton in every view
        if v is None:
            continue
        v = cast(v, "IView")
        for c in v.GetVisibleComponents() or []:
            c = cast(c, "IComponent2")
            if os.path.basename(c.GetPathName()).startswith("RV-000"):
                c.Select4(False, None, False)
                v.HideComponent2() if hasattr(v, "HideComponent2") else None
    iso = cast(views[3], "IView")
    bom = iso.InsertBomTable4(False, 0.37, 0.405, K.swBOMConfigurationAnchor_TopLeft, K.swBomType_PartsOnly, "",
                              os.path.join(LANG, "bom-standard.sldbomtbt"), False, 0, False)
    print("   BOM table:", "OK" if bom is not None else "failed")
    try:
        doc.ClearSelection2(True)
        iso_name = iso.GetName2()
        doc.Extension.SelectByID2(iso_name, "DRAWINGVIEW", 0, 0, 0, False, 0, None, 0)
        opts = dwg.CreateAutoBalloonOptions()
        dwg.AutoBalloon5(opts)
    except Exception as e:
        print(f"   (balloons skipped: {e})")
    doc.ForceRebuild3(False)
    out = os.path.join(OUT, "RV-100_RoamView.SLDDRW")
    if os.path.exists(out):
        os.remove(out)
    doc.SaveAs3(out, 0, K.swSaveAsOptions_Silent)
    pdf = os.path.join(OUT, "PDF", "RV-100_RoamView.pdf")
    if os.path.exists(pdf):
        os.remove(pdf)
    doc.SaveAs3(pdf, 0, K.swSaveAsOptions_Silent | K.swSaveAsOptions_Copy)
    sw.CloseDoc(doc.GetTitle())
    return os.path.exists(out), os.path.exists(pdf)


def main():
    sw = app()
    want = sys.argv[1:]
    made = sorted(glob.glob(os.path.join(ROOT, "3DPrinted", "RV-*.SLDPRT"))
                  + glob.glob(os.path.join(ROOT, "SheetMetal_Weldments", "*", "RV-*.SLDPRT")))
    for path in made:
        if want and not any(os.path.basename(path).startswith(w) for w in want):
            continue
        try:
            base, den, ok = part_drawing(sw, path)
            print(f"{'OK  ' if all(ok) else 'FAIL'} {base:38s} {den:5s} slddrw={ok[0]} pdf={ok[1]}")
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL {os.path.basename(path)}: {e}")
        sw.CloseAllDocuments(True)
    if not want or "RV-100" in want:
        try:
            print("RV-100 assembly drawing:", assembly_drawing(sw))
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL RV-100 drawing: {e}")
        sw.CloseAllDocuments(True)


if __name__ == "__main__":
    main()
