"""Export RV-100 as one binary STL per component, in the assembly frame (mm, not shifted to positive space),
for fov_check.py. Old files in Simulation/FOV/parts are moved to parts_old/ first.

    python export_fov_stl.py
"""
import glob
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import K, ROOT, app, cast  # noqa: E402

OUT = os.path.join(ROOT, "Simulation", "FOV", "parts")


def main():
    sw = app()
    sw.CloseAllDocuments(True)                      # RV-100 open as a sub-assembly makes SaveAs3 throw
    old =os.path.join(os.path.dirname(OUT), "parts_old")
    os.makedirs(old, exist_ok=True)
    for f in glob.glob(os.path.join(OUT, "*.STL")):
        shutil.move(f, os.path.join(old, os.path.basename(f)))
    sw.SetUserPreferenceIntegerValue(K.swSTLQuality, K.swSTLQuality_Fine)
    sw.SetUserPreferenceToggle(K.swSTLBinaryFormat, True)
    sw.SetUserPreferenceToggle(K.swSTLComponentsIntoOneFile, False)
    sw.SetUserPreferenceToggle(K.swSTLDontTranslateToPositive, True)
    sw.SetUserPreferenceIntegerValue(K.swExportStlUnits, K.swMM)
    path = os.path.join(ROOT, "Assemblies", "RV-100_RoamView.SLDASM")
    res = sw.OpenDoc6(path, K.swDocASSEMBLY, K.swOpenDocOptions_Silent, "", 0, 0)
    doc = cast(res[0] if isinstance(res, tuple) else res, "IModelDoc2")
    doc.SaveAs3(os.path.join(OUT, "RV-100.STL"), 0, K.swSaveAsOptions_Silent | K.swSaveAsOptions_Copy)
    sw.CloseDoc(doc.GetTitle())
    files = glob.glob(os.path.join(OUT, "*.STL"))
    print(f"{len(files)} component STLs in {OUT}")


if __name__ == "__main__":
    main()
