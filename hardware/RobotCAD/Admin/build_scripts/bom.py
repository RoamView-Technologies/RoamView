"""Bill of materials from the RV-100 assembly tree + the hardware that is not modelled.

Writes Drawings/RV-100_BOM.csv (one row per unique part, quantity counted over the whole tree) with the
Source split: PURCHASED (COTS), MANUFACTURED (PRINT / SHEET / WELD), RESERVED (ENVELOPE: to buy or design),
and HARDWARE (estimated, not modelled).
"""
import collections
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from bom_purpose import PURPOSE  # noqa: E402
from rv_sw import K, ROOT, app, cast  # noqa: E402

TOP = os.path.join(ROOT, "Assemblies", "RV-100_RoamView.SLDASM")
GROUP = {"COTS": "PURCHASED", "PRINT": "MANUFACTURED - 3D print", "SHEET": "MANUFACTURED - sheet metal",
         "WELD": "MANUFACTURED - weldment", "ENVELOPE": "RESERVED (buy / design)"}

# hardware not modelled (estimate from the design; confirm at assembly)
HARDWARE = [
    ("HW-01", "Heat-set insert M2.5 (Pi 5 x4, LiDAR x3)", 7),
    ("HW-02", "Heat-set insert M3 (OAK wedge x2, driver x4, charge pads x2)", 8),
    ("HW-03", "Heat-set insert M4 (ESP32 board x4, rear caster riser x4)", 8),
    ("HW-04", "2020 inside corner bracket + M4 T-nut set (frame joints)", 24),
    ("HW-05", "M4 x 8 SHCS + M4 slot-6 T-nut (pan, deck, skins, guides, brackets, caster blocks)", 90),
    ("HW-06", "M5 x 12 SHCS (head to post bores)", 2),
    ("HW-07", "M3 x 6 SHCS (gearbox face to motor bracket, 6 per motor)", 12),
    ("HW-08", "M4 x 8 SHCS (wheel to Sonic Hub, 4 per wheel)", 8),
    ("HW-09", "M4 x 10 SHCS (caster plates)", 8),
    ("HW-10", "M4 shoulder bolt 4 x 20 (front caster guide pins)", 2),
    ("HW-12", "M2.5 x 6 self-tapping screw (cliff sensor boards, 2 per cradle)", 8),
    ("HW-19", "M4 x 12 SHCS + M4 slot-6 T-nut (cliff cradles through the end skins into the MidCross)", 8),
    ("HW-13", "M2 x 10 screw + washer + nut (bumper switches to their slotted brackets, 2 each)", 4),
    ("HW-20", "M4 x 8 SHCS + M4 slot-6 T-nut (bumper switch brackets through the front skin)", 2),
    ("HW-21", "Bumper float: 2 x shoulder bolt d5, 4 mm travel, + 2 light return springs (guide pins at +-150)", 2),
    ("HW-14", "Neodymium magnet d6 x 3 (dock detect)", 1),
    ("HW-15", "XT60 panel-mount female + bracket (drawer blind mate) - bracket TBD", 1),
    ("HW-16", "Cable grommet d25 / d16 (deck ports)", 3),
    ("HW-17", "Addressable LED strip, ~260 mm (status light slot)", 1),
    ("HW-18", "Drawer latch / magnetic catch (validation recommendation)", 1),
]


def props(doc):
    cpm = doc.Extension.CustomPropertyManager("")
    out = {}
    for k in ("PartNo", "Description", "Source", "Vendor", "VendorPN", "Material", "DimSource", "Mass_g"):
        try:
            r = cpm.Get6(k, False)
            out[k] = r[1] if isinstance(r, tuple) else ""
        except Exception:
            out[k] = ""
    m = cast(doc.Extension.CreateMassProperty(), "IMassProperty")
    out["mass_g"] = round(m.Mass * 1000, 1)
    return out


def main():
    sw = app()
    res = sw.OpenDoc6(TOP, K.swDocASSEMBLY, K.swOpenDocOptions_Silent, "", 0, 0)
    doc = cast(res[0] if isinstance(res, tuple) else res, "IModelDoc2")
    count, info = collections.Counter(), {}
    for c in cast(doc, "IAssemblyDoc").GetComponents(False) or []:
        c = cast(c, "IComponent2")
        md = c.GetModelDoc2()
        if md is None or c.IsSuppressed():
            continue
        md = cast(md, "IModelDoc2")
        if md.GetType() != K.swDocPART:
            continue
        path = c.GetPathName()
        count[path] += 1
        if path not in info:
            info[path] = props(md)
    sw.CloseDoc(doc.GetTitle())
    rows = []
    for path, q in count.items():
        p = info[path]
        if p["Source"] in ("LAYOUT", "REFERENCE"):
            continue
        rows.append([GROUP.get(p["Source"], p["Source"]), p["PartNo"], os.path.splitext(os.path.basename(path))[0].split("_", 1)[1], p["Description"],
                     PURPOSE.get(p["PartNo"], ""), q, p["Vendor"], p["VendorPN"], p["Material"], p["mass_g"], round(p["mass_g"] * q, 1),
                     p["DimSource"]])
    rows.sort(key=lambda r: (r[0], r[1]))
    rows += [["HARDWARE (estimate, not modelled)", pn, "", desc, PURPOSE.get(pn, ""), q, "", "", "", "", "", "estimate"]
             for pn, desc, q in HARDWARE]
    out = os.path.join(ROOT, "Drawings", "RV-100_BOM.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Group", "PartNo", "Name", "Description", "Purpose", "Qty", "Vendor", "VendorPN", "Material",
                    "Unit_mass_g", "Total_mass_g", "Notes / dimension source"])
        w.writerows(rows)
    by = collections.defaultdict(lambda: [0, 0, 0.0])
    for r in rows:
        by[r[0]][0] += 1
        by[r[0]][1] += r[5]
        by[r[0]][2] += r[10] or 0
    for g, (n, q, m) in sorted(by.items()):
        print(f"{g:38s} {n:3d} line items, {q:4d} pcs, {m / 1000:5.2f} kg")
    print(f"total modelled mass {sum(r[10] or 0 for r in rows) / 1000:.2f} kg -> {out}")
    missing = [r[1] for r in rows if not r[4]]
    if missing:
        print("WARNING: no Purpose text in bom_purpose.py for", ", ".join(missing))
    try:
        from bom_xlsx import write_xlsx
        print("->", write_xlsx(out))
    except ImportError:
        print("openpyxl not installed here: run `python bom_xlsx.py` from a Python that has it for the .xlsx")


if __name__ == "__main__":
    main()
