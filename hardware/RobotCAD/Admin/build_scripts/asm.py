"""Assembly helpers for RoamView. Modules mate interface-frame to interface-frame (coordinate-system mates),
so swapping a component only needs the replacement to carry the same frame names.
Moving joints (wheel spin, caster swivel, sprung caster travel, drawer slide) use axis/plane mates."""
import os

from rv_sw import K, MM, ROOT, Part, cast


class Asm(Part):
    @classmethod
    def new(cls, sw):
        tmpl = sw.GetUserPreferenceStringValue(K.swDefaultTemplateAssembly)
        doc = sw.NewDocument(tmpl, 0, 0, 0)
        a = cls(sw, doc)
        a.ext.SetUserPreferenceInteger(K.swUnitSystem, 0, K.swUnitSystem_MMGS)
        a.asm = cast(a.doc, "IAssemblyDoc")
        a.title = a.doc.GetTitle()
        a.opened = []
        return a

    # ---------- components ----------
    def insert(self, path):
        """Insert a part/assembly file at the origin, identity orientation."""
        if not os.path.isabs(path):
            path = os.path.join(ROOT, path)
        path = os.path.normpath(path)
        typ = K.swDocASSEMBLY if path.upper().endswith("ASM") else K.swDocPART
        res = self.sw.OpenDoc6(path, typ, K.swOpenDocOptions_Silent, "", 0, 0)
        if (res[0] if isinstance(res, tuple) else res) is None:
            raise RuntimeError(f"cannot open {path}: {res}")
        self.opened.append(path)
        self.sw.ActivateDoc3(self.title, False, 0, None)
        comp = cast(self.asm.AddComponent5(path, 0, "", False, "", 0, 0, 0), "IComponent2")
        if comp is None:
            raise RuntimeError(f"AddComponent5 failed for {path}")
        mu = cast(self.sw.GetMathUtility(), "IMathUtility")
        import pythoncom
        import win32com.client as w32
        ident = w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0])
        comp.Transform2 = cast(mu.CreateTransform(ident), "IMathTransform")
        return comp

    def fix(self, comp):
        self.clear()
        comp.Select4(False, None, False)
        self.asm.FixComponent()
        self.clear()

    def flexible(self, comp):
        """Solve a subassembly as flexible so its internal joints (wheel spin, swivel, spring) move here."""
        self.clear()
        comp.Select4(False, None, False)
        ok = self.asm.CompConfigProperties6(K.swComponentFullyResolved, K.swComponentFlexibleSolving, True, False, "",
                                           False, False, 0)
        self.clear()
        if not ok or comp.Solving != K.swComponentFlexibleSolving:
            raise RuntimeError(f"could not make {comp.Name2} flexible")

    def close_sources(self):
        for path in self.opened:
            title = os.path.basename(path)
            doc = self.sw.GetOpenDocumentByName(path)
            if doc is not None and cast(doc, "IModelDoc2").GetTitle() != self.title:
                try:
                    self.sw.CloseDoc(cast(doc, "IModelDoc2").GetTitle())
                except Exception:
                    pass
        self.opened = []

    # ---------- selection ----------
    def sel(self, comp, feat, typ, append=False, mark=0):
        """Select a feature of a component (comp=None -> the assembly's own feature)."""
        name = f"{feat}@{self.title}" if comp is None else f"{feat}@{comp.Name2}@{self.title}"
        ok = self.ext.SelectByID2(name, typ, 0, 0, 0, append, mark, None, 0)
        if not ok:
            raise RuntimeError(f"cannot select {typ} {name}")

    # ---------- mates ----------
    def _mate(self, mtype, align, dist=0.0, hi=0.0, lo=0.0, name=None):
        res = self.asm.AddMate5(mtype, align, False, dist, hi, lo, 1, 1, 0, 0, 0, False, False, 0)
        mate = res[0] if isinstance(res, tuple) else res
        err = res[1] if isinstance(res, tuple) else None
        self.clear()
        if mate is None:
            raise RuntimeError(f"mate failed (type {mtype}, err {err})")
        if name:
            last = None
            for f in self.features():
                if f.GetTypeName2() == "MateGroup":
                    m = cast(f.GetFirstSubFeature(), "IFeature")
                    while m is not None:
                        last, m = m, cast(m.GetNextSubFeature(), "IFeature")
            if last is not None:
                last.Name = name
        return mate

    def cs_mate(self, comp_a, cs_a, comp_b, cs_b, name=None):
        """Lock two interface frames together (all six DOF)."""
        self.clear()
        self.sel(comp_a, cs_a, "COORDSYS")
        self.sel(comp_b, cs_b, "COORDSYS", append=True)
        return self._mate(K.swMateCOORDINATE, K.swMateAlignALIGNED, name=name)

    def plane_mate(self, comp_a, pl_a, comp_b, pl_b, align=None, name=None):
        self.clear()
        self.sel(comp_a, pl_a, "PLANE")
        self.sel(comp_b, pl_b, "PLANE", append=True)
        return self._mate(K.swMateCOINCIDENT, K.swMateAlignALIGNED if align is None else align, name=name)

    def axis_mate(self, comp_a, ax_a, comp_b, ax_b, name=None):
        self.clear()
        self.sel(comp_a, ax_a, "AXIS")
        self.sel(comp_b, ax_b, "AXIS", append=True)
        return self._mate(K.swMateCOINCIDENT, K.swMateAlignALIGNED, name=name)

    def limit_mate(self, comp_a, pl_a, comp_b, pl_b, lo_mm, hi_mm, nominal_mm=0.0, name=None):
        """Advanced limit-distance mate between two parallel planes."""
        self.clear()
        self.sel(comp_a, pl_a, "PLANE")
        self.sel(comp_b, pl_b, "PLANE", append=True)
        return self._mate(K.swMateDISTANCE, K.swMateAlignALIGNED, nominal_mm * MM, hi_mm * MM, lo_mm * MM, name=name)

    # ---------- checks ----------
    def frame_world(self, comp, cs_name):
        """World (assembly) ROS position of a component's coordinate system."""
        t = cast(comp.Transform2, "IMathTransform").ArrayData
        doc = cast(comp.GetModelDoc2(), "IModelDoc2")
        m = cast(doc.Extension.GetCoordinateSystemTransformByName(cs_name), "IMathTransform").ArrayData
        p = [m[9], m[10], m[11]]
        w = [p[0] * t[0] + p[1] * t[3] + p[2] * t[6] + t[9],
             p[0] * t[1] + p[1] * t[4] + p[2] * t[7] + t[10],
             p[0] * t[2] + p[1] * t[5] + p[2] * t[8] + t[11]]
        return (w[2] / MM, w[0] / MM, w[1] / MM)

    def unsolved(self):
        """Mates with errors or warnings."""
        bad = []
        f = cast(self.doc.FirstFeature(), "IFeature")
        while f is not None:
            if f.GetTypeName2() == "MateGroup":
                m = cast(f.GetFirstSubFeature(), "IFeature")
                while m is not None:
                    code = m.GetErrorCode2()
                    if code[0] != 0:
                        bad.append((m.Name, code))
                    m = cast(m.GetNextSubFeature(), "IFeature")
            f = cast(f.GetNextFeature(), "IFeature")
        return bad

    def save_asm(self, path):
        if not os.path.isabs(path):
            path = os.path.join(ROOT, path)
        path = os.path.normpath(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.doc.ForceRebuild3(False)
        err = self.doc.SaveAs3(path, 0, K.swSaveAsOptions_Silent)
        if err not in (0, K.swFileSaveWithRebuildError):
            raise RuntimeError(f"SaveAs3 {err} for {path}")
        self.path = path
        return path
