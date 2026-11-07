"""Give every RoamView part a realistic mass (mass override, COM stays at the geometric centroid).

COTS / envelopes : custom property Mass_g (datasheet or estimate)
Weldments        : 2020 at 0.46 kg/m + bracket/T-nut allowance (fixed values below)
Sheet metal      : volume x 2.68 g/cm3 (5052), brass pad 8.5
Printed          : volume x effective density (thin shells/skins print near-solid, blocks at ~55 % infill)
Writes Simulation/MassProps/mass_table.csv.
"""
import csv
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, ROOT, app, cast  # noqa: E402

FIXED_G = {"RV-410": 2250.0, "RV-420": 450.0}          # 4.41 m / 0.75 m of 2020 + plates + hardware
DENSITY = {"RV-435": 8.5}                               # brass
SHEET = 2.68
PRINT_SHELL, PRINT_BLOCK, TPU = 1.15, 0.70, 0.73
SHELLS = {"RV-301", "RV-302", "RV-313", "RV-314", "RV-315"}


def prop(doc, name):
    cpm = doc.Extension.CustomPropertyManager("")
    try:
        r = cpm.Get6(name, False)
        return r[1] if isinstance(r, tuple) else None
    except Exception:
        r = cpm.Get4(name, False)
        return r[1] if isinstance(r, tuple) else None


def main():
    sw = app()
    sw.CloseAllDocuments(True)
    rows = []
    pats = ["3DPrinted/RV-*.SLDPRT", "SheetMetal_Weldments/*/RV-*.SLDPRT", "COTS/*/RV-*.SLDPRT"]
    for pat in pats:
        for path in sorted(glob.glob(os.path.join(ROOT, pat))):
            pn, name = os.path.basename(path)[:-7].split("_", 1)        # "RV-434_MotorBracket.SLDPRT"
            r = sw.OpenDoc6(path, K.swDocPART, K.swOpenDocOptions_Silent, "", 0, 0)
            # use the document OpenDoc6 returns, never ActiveDoc: if an assembly that uses this part is still open,
            # ActiveDoc is that assembly and the override would land on it
            doc = cast(r[0] if isinstance(r, tuple) else r, "IModelDoc2")
            if doc is None or os.path.normcase(doc.GetPathName()) != os.path.normcase(path):
                raise RuntimeError(f"opened the wrong document for {path}")
            doc.ForceRebuild3(False)                    # a freshly built part can report a stale volume until rebuilt
            m = cast(doc.Extension.CreateMassProperty(), "IMassProperty")
            m.OverrideMass = False
            m = cast(doc.Extension.CreateMassProperty(), "IMassProperty")
            vol_cm3 = m.Volume * 1e6
            src = prop(doc, "Source") or ""
            if pn in FIXED_G:
                g, basis = FIXED_G[pn], "2020 0.46 kg/m + hardware"
            elif src in ("COTS", "ENVELOPE"):
                g, basis = float(prop(doc, "Mass_g")), "datasheet / estimate (Mass_g)"
            elif pn in DENSITY:
                g, basis = vol_cm3 * DENSITY[pn], f"{DENSITY[pn]} g/cm3"
            elif src == "SHEET":
                g, basis = vol_cm3 * SHEET, "5052 2.68 g/cm3"
            elif pn == "RV-305":
                g, basis = vol_cm3 * TPU, "TPU 0.73 g/cm3 eff."
            else:
                d = PRINT_SHELL if pn in SHELLS else PRINT_BLOCK
                g, basis = vol_cm3 * d, f"PETG {d} g/cm3 eff."
            m.OverrideMass = True
            m.SetOverrideMassValue(g / 1000.0, K.swThisConfiguration, None)
            doc.ForceRebuild3(False)
            ok = doc.Save3(K.swSaveAsOptions_Silent, 0, 0)[0]
            rows.append([pn, name, src, round(vol_cm3, 1), round(g, 1), basis, ok])
            sw.CloseDoc(doc.GetTitle())
            print(f"{pn} {name:34s} {src:9s} vol {vol_cm3:8.1f} cm3 -> {g:8.1f} g  ({basis})")
    out = os.path.join(ROOT, "Simulation", "MassProps", "mass_table.csv")
    with open(out, "w", newline="") as f:
        csv.writer(f).writerows([["PartNo", "Name", "Source", "Volume_cm3", "Mass_g", "Basis", "Saved"]] + rows)
    print(f"\n{len(rows)} parts, total part masses (single instances) {sum(r[4] for r in rows) / 1000:.2f} kg -> {out}")


if __name__ == "__main__":
    main()
