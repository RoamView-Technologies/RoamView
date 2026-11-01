# RoamView interconnect model, Rev B (layout 2)

Source of truth for the Rev B part placements and cable routes (DESIGN_RULES.md section 1).
Python 3.11 with numpy, scipy, shapely, numba and matplotlib.

| File | What it does |
|---|---|
| model_b.py | Every part (prims), port, cable and keep-out. Edit this, then rerun the rest. |
| rules_b.py | Design-rule check: spacing, finger zones, footprint, LiDAR band, OAK view, battery path, wheel wells, suspension sweep, caster swing, plate holes. `python rules_b.py` must print 0 issues. |
| plot_b.py | Top views of each level with finger zones and pass-throughs -> layout_b.png |
| route_b.py, build_b.py | 3D cable router and strict checks -> routes_b.npy. Must print 0 clearance issues and 0 overlaps. |
| export_b.py | Writes model_b.json for the web preview and the harness totals. |
| cog_b.py | Mass and centre-of-gravity estimate, tip-over margins and the cliff-stop check. |
| cad_spec_b.py | Writes RoamView_RevB_spec.json for the SOLIDWORKS build (`old-designs/RevB/RoamView_Build_RevB.py`). |
| RoamView_Harness_CutList_RevB.csv | Harness schedule: every cable's ends, wire, routed and cut length, connectors and pins. |

Order: `python rules_b.py && python plot_b.py && python build_b.py && python export_b.py && python cad_spec_b.py`, then copy RoamView_RevB_spec.json to `old-designs/RevB/` and run `python RoamView_Build_RevB.py` there.
