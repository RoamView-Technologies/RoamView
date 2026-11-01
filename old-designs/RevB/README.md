# RoamView chassis, Rev B (layout 2) - SOLIDWORKS model

Built by `RoamView_Build_RevB.py` from `RoamView_RevB_spec.json`. The Rev A files in `old-designs/` are not touched, and every
Rev B file name ends in `_RevB`, so both revisions can be open in SOLIDWORKS at the same time.

| File | What it is |
|---|---|
| `RoamView_Chassis_RevB.SLDASM` (+ `.STEP`, PNG views) | Full robot: both decks, 48 parts, harness. Placed by coordinates and fixed, like Rev A. |
| `RoamView_Baseplate_RevB.SLDPRT` (+ `.STEP`, PNG) | Lower deck RV-CH-001B, 6061-T6, 320 x 260 x 6 mm |
| `RoamView_TopDeck_RevB.SLDPRT` (+ `.STEP`, PNG) | Top deck RV-CH-002B, 6061-T6, 320 x 260 x 3 mm |
| `Components/*_RevB.SLDPRT` | 40 part files (48 placed parts): envelopes at datasheet size. Parts marked `Placeholder = yes` in their properties are still to be chosen. |
| `RoamView_Harness_RevB.SLDPRT` | 62 swept conductors (45 cables), one body each, named by cable ID (P01+, P01-, D06, ...) |
| `RoamView_build_log_RevB.txt` | Every feature, volume, box check and interference result from the last build |
| `RoamView_build_report_RevB.json` | The same checks in machine-readable form |

## Rebuild

1. Change the design in `old-designs/Interconnect_RevB/model_b.py`, then run `python rules_b.py` (0 issues) and `python build_b.py` (0 clearance issues, 0 overlaps) there.
2. `python cad_spec_b.py` there writes `RoamView_RevB_spec.json`; copy it into this folder.
3. Here: `python RoamView_Build_RevB.py` with SOLIDWORKS open or closed. Needs Python 3 with pywin32 (the Anaconda Python has it). About 7 minutes.
   `python RoamView_Build_RevB.py plates|parts|harness|assembly` rebuilds one stage.

## Frame

+X forward, +Y up, +Z robot right, millimetres. Assembly origin = the floor under the drive-axle centre (ROS `base_footprint`).
Each component's origin is its bottom centre, so the same file is reused for identical parts (4 posts, 4 cliff sensors, 2 wheels, 2 springs).

## Known and intended interferences

The build log's interference check (parts only, before the harness goes in) should list only these, all intended:
- each drive motor shaft in its wheel hub (the wheel envelope has no bore);
- each pivot pin in its hanger bushing (the hanger envelope has no bore);
- each spring's guide bolt through its drive-module tab (the tab has no hole).

The harness is left out of the check, because every conductor ends inside its connector.
