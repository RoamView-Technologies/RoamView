import os, sys, inspect
sys.argv = ["probe"]
exec(open("RoamView_Turntable_RevB.py", encoding="utf-8").read().split('HERE = os.path.dirname')[0])
import re
src = inspect.getsource(SW.IModelView)
for name in ("RotateAboutAxis", "RotateAboutCenter", "RotateAboutPoint", "TranslateBy", "ZoomByFactor", "Transform"):
    for mm in re.finditer(r"def %s\(.*?\n(?:.*?\n){0,6}" % name, src):
        print(mm.group(0).rstrip()[:600]); print("-" * 50)
