"""Re-read RV_globals.txt into linked RoamView documents, rebuild, re-audit sketches, save.
    python update_from_globals.py Layout/*.SLDPRT 3DPrinted/*.SLDPRT ..."""
import glob, os, sys
sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import GLOBALS, K, ROOT, Part, app, cast, read_globals

G = read_globals()
stack = G["HEAD_X1"] + G["BEZEL_T"] + G["WEDGE_MID"] + G["OAK_D"]
if abs(stack - G["LENS_X"]) > 0.05:
    print(f"WARNING: LENS_X {G['LENS_X']} != head/bezel/wedge/OAK stack-up {stack:.2f} - update RV_globals.txt")
sw = app()
for pat in sys.argv[1:]:
    for path in sorted(glob.glob(os.path.join(ROOT, pat))):
        typ = K.swDocASSEMBLY if path.upper().endswith("ASM") else K.swDocPART
        res = sw.OpenDoc6(path, typ, K.swOpenDocOptions_Silent, "", 0, 0)
        doc = res[0] if isinstance(res, tuple) else res
        if doc is None:
            print(f"{os.path.basename(path):40s} COULD NOT OPEN {res}")
            continue
        p = Part(sw, doc)
        if p.eq.LinkToFile:                      # re-link: imports globals added since the part was made
            p.eq.LinkToFile = False
            p.eq.FilePath = GLOBALS
            p.eq.LinkToFile = True
            p.eq.AutomaticSolveOrder = True
            p.eq.UpdateValuesFromExternalEquationFile()
        for _ in range(2):
            p.eq.EvaluateAll(); p.doc.ForceRebuild3(False)
        sk = {f.Name: cast(f.GetSpecificFeature2(), "ISketch").GetConstrainedStatus()
              for f in p.features() if f.GetTypeName2() == "ProfileFeature"}
        bad = [k for k, v in sk.items() if v != K.swFullyConstrained]
        errs = p.errors()
        ok = p.doc.Save3(K.swSaveAsOptions_Silent, 0, 0)[0]
        print(f"{os.path.basename(path):40s} linked={p.eq.LinkToFile} sketches={len(sk)} not_fd={bad} errors={errs} saved={ok}")
        sw.CloseDoc(p.doc.GetTitle())
