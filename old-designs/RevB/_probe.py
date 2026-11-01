import os, sys
sys.argv = ["probe"]
exec(open("RoamView_Turntable_RevB.py", encoding="utf-8").read().split('HERE = os.path.dirname')[0])
for iface in ("IModelView", "IModelDoc2"):
    names = sorted(n for n in dir(getattr(SW, iface)) if any(k in n for k in ("Rot", "rot", "Trans", "Zoom", "Scale", "Orient", "Camera", "View")))
    print("==", iface, "==")
    print(", ".join(names))
