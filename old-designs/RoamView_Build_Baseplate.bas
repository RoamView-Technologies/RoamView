Attribute VB_Name = "RoamViewBaseplate"
' =====================================================================
'  RoamView_Build_Baseplate  (SOLIDWORKS VBA macro)
'  ENGR 490 Capstone - RoamView lower chassis deck, Rev A first pass
'
'  Builds RoamView_Baseplate.SLDPRT from scratch with exact geometry,
'  saves it, and exports STEP / STL / PNG next to it, with a build log.
'  Safe to re-run: it closes the old copy, rebuilds and overwrites.
'
'  Axes (top view, all sketches on Top Plane):
'     +X = robot forward, +Y(sketch) = robot left, origin = drive-axle centre
'     Model +Y = up; the plate bottom face lies on the Top Plane.
'     ROS base_link: x = model X, y = -model Z, z = model Y
'  All dimensions below are millimetres.
' =====================================================================
Option Explicit

Const OUT_DIR As String = "C:\Users\sanom\OneDrive - Concordia University - Canada\ENGR 490  - Capstone\RoamView\old-designs\"
Const PART_NAME As String = "RoamView_Baseplate"

' --- Plate -------------------------------------------------------------
Const PLATE_L As Double = 320      ' length along X (forward)
Const PLATE_W As Double = 260      ' width along Y
Const PLATE_T As Double = 6        ' thickness (3/16 in = 4.76 saves ~0.25 kg)
Const CORNER_R As Double = 20      ' corner radius, concentric with deck posts

' --- Wheel wells for 100 mm dia x ~26 mm wide drive wheels --------------
Const WELL_HALF As Double = 62     ' well spans X = -62 .. +62
Const WELL_DEPTH As Double = 42    ' cut in from each side edge
Const WELL_R As Double = 6         ' inner corner radius

' --- Clearance hole sizes ----------------------------------------------
Const M3 As Double = 3.4
Const M4 As Double = 4.5
Const M5 As Double = 5.5

Dim swApp As SldWorks.SldWorks
Dim swModel As SldWorks.ModelDoc2
Dim swPart As SldWorks.PartDoc
Dim swExt As SldWorks.ModelDocExtension
Dim swSkMgr As SldWorks.SketchManager
Dim swFeatMgr As SldWorks.FeatureManager
Dim logF As Integer
Dim curStep As String

Sub main()
    Dim fSk As SldWorks.Feature, fOK As SldWorks.Feature
    Dim sv As Variant, xv As Variant, cx As Variant, dx As Variant, yv As Variant
    Dim i As Long, tmpl As String, db As String
    Dim mp As Variant, st As Long

    logF = FreeFile
    Open OUT_DIR & PART_NAME & "_build_log.txt" For Output As #logF
    On Error GoTo EH

    curStep = "connect"
    Set swApp = Application.SldWorks
    WLog "RoamView baseplate build - SOLIDWORKS " & swApp.RevisionNumber

    curStep = "close old copy"
    CloseIfOpen OUT_DIR & PART_NAME & ".SLDPRT"

    curStep = "new part"
    tmpl = swApp.GetUserPreferenceStringValue(swUserPreferenceStringValue_e.swDefaultTemplatePart)
    WLog "Template: " & tmpl
    Set swModel = swApp.NewDocument(tmpl, 0, 0, 0)
    If swModel Is Nothing Then Set swModel = swApp.NewPart
    If swModel Is Nothing Then Err.Raise vbObjectError + 513, , "Could not create a new part"
    Set swPart = swModel
    Set swExt = swModel.Extension
    Set swSkMgr = swModel.SketchManager
    Set swFeatMgr = swModel.FeatureManager
    swExt.SetUserPreferenceInteger swUserPreferenceIntegerValue_e.swUnitSystem, 0, swUnitSystem_e.swUnitSystem_MMGS
    WLog "New document: " & swModel.GetTitle & " (type " & swModel.GetType & ")"
    swApp.ActivateDoc3 swModel.GetTitle, False, swRebuildOnActivation_e.swDontRebuildActiveDoc, 0
    LogTree

    ' 1. Plate outline with wheel wells ---------------------------------
    curStep = "plate outline"
    StartSketch
    PlateOutline
    Set fSk = EndSketch("SK_Plate")
    curStep = "plate extrude"
    swModel.ClearSelection2 True
    fSk.Select2 False, 0
    Set fOK = swFeatMgr.FeatureExtrusion2(True, False, False, 0, 0, Mt(PLATE_T), 0, False, False, False, False, 0, 0, False, False, False, False, True, True, True, 0, 0, False)
    If fOK Is Nothing Then Err.Raise vbObjectError + 514, , "Plate extrusion failed (profile not closed?)"
    fOK.Name = "Plate"
    LogBox "Plate"

    ' 2. Drive-motor bracket slots, under the plate on the axle line -----
    '    M3, slotted along Y so the wheel-to-well clearance can be set.
    curStep = "motor mounts"
    StartSketch
    For Each sv In Array(1, -1)
        For Each xv In Array(-12.5, 12.5)
            Slot xv, sv * 64, xv, sv * 76, M3
        Next
    Next
    Set fSk = EndSketch("SK_MotorMounts")
    CutThroughAll fSk, "Cut_MotorMounts_M3"

    ' 3. Caster mounts front (+X) and rear (-X), M4 slots ----------------
    curStep = "caster mounts"
    StartSketch
    For Each cx In Array(130, -130)
        For Each dx In Array(-17.5, 17.5)
            For Each sv In Array(1, -1)
                Slot cx + dx, sv * 20, cx + dx, sv * 28, M4
            Next
        Next
    Next
    Set fSk = EndSketch("SK_CasterMounts")
    CutThroughAll fSk, "Cut_CasterMounts_M4"

    ' 4. Battery bay over the axle: 25 mm strap slots + tray holes -------
    curStep = "battery bay"
    StartSketch
    For Each xv In Array(-22, 22)
        For Each sv In Array(1, -1)
            Slot xv - 14, sv * 54, xv + 14, sv * 54, 4
        Next
    Next
    For Each xv In Array(-38, 38)
        For Each sv In Array(1, -1)
            Hole xv, sv * 40, M3
        Next
    Next
    Set fSk = EndSketch("SK_BatteryBay")
    CutThroughAll fSk, "Cut_BatteryBay"

    ' 5. Cable pass-throughs front and rear of the battery ---------------
    curStep = "cable pass-throughs"
    StartSketch
    Slot 58, -20, 58, 20, 12
    Slot -58, -20, -58, 20, 12
    Set fSk = EndSketch("SK_CablePassThru")
    CutThroughAll fSk, "Cut_CablePassThru"

    ' 6. Electronics grid, M3 on 20 mm pitch, front and rear zones -------
    curStep = "electronics grid"
    StartSketch
    For Each xv In Array(-100, -80, 80, 100)
        For i = -5 To 5
            Hole xv, i * 20, M3
        Next
    Next
    Set fSk = EndSketch("SK_ElecGrid_M3")
    CutThroughAll fSk, "Cut_ElecGrid_M3"

    ' 7. Top-deck posts: 2020 extrusion, M5 into the centre bore ---------
    curStep = "deck posts"
    StartSketch
    For Each xv In Array(140, -140)
        For Each yv In Array(110, -110)
            Hole xv, yv, M5
        Next
    Next
    Set fSk = EndSketch("SK_DeckPosts_M5")
    CutThroughAll fSk, "Cut_DeckPosts_M5"

    ' 8. Downward cliff-sensor windows at all four corners --------------
    curStep = "cliff sensors"
    StartSketch
    For Each xv In Array(145, -145)
        For Each yv In Array(88, -88)
            Hole xv, yv, 12
        Next
    Next
    Set fSk = EndSketch("SK_CliffSensors")
    CutThroughAll fSk, "Cut_CliffSensorWindows"

    ' 9. Dock charging contacts at the front, 2 x M4 per contact --------
    curStep = "charging contacts"
    StartSketch
    For Each yv In Array(40, 60, -40, -60)
        Hole 150, yv, M4
    Next
    Set fSk = EndSketch("SK_ChargeContacts")
    CutThroughAll fSk, "Cut_ChargeContacts_M4"

    ' Material, properties, checks ----------------------------------------
    curStep = "material"
    swPart.SetMaterialPropertyName2 "Default", "SOLIDWORKS Materials", "6061-T6 (SS)"
    WLog "Material: " & swPart.GetMaterialPropertyName2("Default", db) & " [" & db & "]"

    curStep = "properties"
    SetProp "Description", "RoamView lower chassis deck (baseplate)"
    SetProp "PartNo", "RV-CH-001"
    SetProp "Revision", "A"
    SetProp "Project", "ENGR 490 RoamView"
    SetProp "Stock", "6061-T6 aluminium plate, 6 mm"
    SetProp "Notes", "Motor and caster patterns are adjustment slots - confirm against purchased parts"

    curStep = "mass properties"
    swModel.ForceRebuild3 False
    mp = swExt.GetMassProperties2(1, st, False)
    If IsArray(mp) Then
        WLog "Mass: " & Format$(mp(5), "0.000") & " kg   Volume: " & Format$(mp(3) * 1000000#, "0.0") & " cm3"
        WLog "Centre of mass (mm): X " & F1(mp(0)) & "  Y " & F1(mp(1)) & "  Z " & F1(mp(2))
    End If
    LogBox "Final"
    LogTree

    curStep = "save"
    swModel.ClearSelection2 True
    swModel.ShowNamedView2 "*Isometric", 7
    swModel.ViewZoomtofit2
    SaveCopy ".SLDPRT", False
    SaveCopy ".STEP", True
    SaveCopy ".STL", True
    If Not SaveCopy("_iso.png", True) Then SaveCopy "_iso.jpg", True
    swModel.ShowNamedView2 "*Top", 5
    swModel.ViewZoomtofit2
    If Not SaveCopy("_top.png", True) Then SaveCopy "_top.jpg", True
    swModel.ShowNamedView2 "*Isometric", 7
    swModel.ViewZoomtofit2

    WLog "BUILD COMPLETE"
    Close #logF
    Exit Sub

EH:
    WLog "*** FAILED at [" & curStep & "]: " & Err.Number & "  " & Err.Description
    On Error Resume Next
    swSkMgr.AddToDB = False
    swSkMgr.DisplayWhenAdded = True
    Close #logF
End Sub

' ---------------------------------------------------------------------
'  Plate outline: rounded rectangle with two wheel wells, one closed
'  loop drawn counter-clockwise (lines + tangent arcs).
' ---------------------------------------------------------------------
Private Sub PlateOutline()
    Dim hx As Double, hy As Double, R As Double
    Dim wx As Double, wy As Double, rw As Double
    hx = PLATE_L / 2: hy = PLATE_W / 2: R = CORNER_R
    wx = WELL_HALF: wy = hy - WELL_DEPTH: rw = WELL_R

    Ln hx, -hy + R, hx, hy - R                          ' front edge
    Ar hx - R, hy - R, hx, hy - R, hx - R, hy, 1        ' front-left corner
    Ln hx - R, hy, wx, hy                               ' left edge (front)
    Ln wx, hy, wx, wy + rw                              ' left well
    Ar wx - rw, wy + rw, wx, wy + rw, wx - rw, wy, -1
    Ln wx - rw, wy, -wx + rw, wy
    Ar -wx + rw, wy + rw, -wx + rw, wy, -wx, wy + rw, -1
    Ln -wx, wy + rw, -wx, hy
    Ln -wx, hy, -hx + R, hy                             ' left edge (rear)
    Ar -hx + R, hy - R, -hx + R, hy, -hx, hy - R, 1     ' rear-left corner
    Ln -hx, hy - R, -hx, -hy + R                        ' rear edge
    Ar -hx + R, -hy + R, -hx, -hy + R, -hx + R, -hy, 1  ' rear-right corner
    Ln -hx + R, -hy, -wx, -hy                           ' right edge (rear)
    Ln -wx, -hy, -wx, -wy - rw                          ' right well
    Ar -wx + rw, -wy - rw, -wx, -wy - rw, -wx + rw, -wy, -1
    Ln -wx + rw, -wy, wx - rw, -wy
    Ar wx - rw, -wy - rw, wx - rw, -wy, wx, -wy - rw, -1
    Ln wx, -wy - rw, wx, -hy
    Ln wx, -hy, hx - R, -hy                             ' right edge (front)
    Ar hx - R, -hy + R, hx - R, -hy, hx, -hy + R, 1     ' front-right corner
End Sub

' ---------------------------------------------------------------------
'  Sketch helpers (inputs in mm, sketch coordinates)
' ---------------------------------------------------------------------
Private Function Mt(ByVal v As Double) As Double
    Mt = v / 1000#
End Function

Private Sub Ln(ByVal x1 As Double, ByVal y1 As Double, ByVal x2 As Double, ByVal y2 As Double)
    swSkMgr.CreateLine Mt(x1), Mt(y1), 0#, Mt(x2), Mt(y2), 0#
End Sub

Private Sub Ar(ByVal xc As Double, ByVal yc As Double, ByVal x1 As Double, ByVal y1 As Double, _
               ByVal x2 As Double, ByVal y2 As Double, ByVal dirn As Integer)
    ' dirn: 1 = counter-clockwise, -1 = clockwise (from point 1 to point 2)
    swSkMgr.CreateArc Mt(xc), Mt(yc), 0#, Mt(x1), Mt(y1), 0#, Mt(x2), Mt(y2), 0#, dirn
End Sub

Private Sub Hole(ByVal x As Double, ByVal y As Double, ByVal dia As Double)
    swSkMgr.CreateCircleByRadius Mt(x), Mt(y), 0#, Mt(dia / 2)
End Sub

Private Sub Slot(ByVal x1 As Double, ByVal y1 As Double, ByVal x2 As Double, ByVal y2 As Double, ByVal w As Double)
    ' Obround slot of width w between centre points (x1,y1) and (x2,y2)
    Dim L As Double, nx As Double, ny As Double, r As Double
    L = Sqr((x2 - x1) ^ 2 + (y2 - y1) ^ 2)
    If L < 0.000001 Then
        Hole x1, y1, w
        Exit Sub
    End If
    nx = -(y2 - y1) / L: ny = (x2 - x1) / L
    r = w / 2
    Ln x1 + r * nx, y1 + r * ny, x2 + r * nx, y2 + r * ny
    Ar x2, y2, x2 + r * nx, y2 + r * ny, x2 - r * nx, y2 - r * ny, -1
    Ln x2 - r * nx, y2 - r * ny, x1 - r * nx, y1 - r * ny
    Ar x1, y1, x1 - r * nx, y1 - r * ny, x1 + r * nx, y1 + r * ny, -1
End Sub

Private Sub StartSketch()
    swModel.ClearSelection2 True
    SelectTopPlane
    swSkMgr.InsertSketch True
    If swSkMgr.ActiveSketch Is Nothing Then Err.Raise vbObjectError + 521, , "Sketch did not start"
    swSkMgr.AddToDB = True
    swSkMgr.DisplayWhenAdded = False
End Sub

' Top Plane is the 2nd reference plane in every standard template; fall back
' to that if the plane has been renamed or the template is localized.
Private Sub SelectTopPlane()
    Dim f As SldWorks.Feature, n As Long
    If swExt.SelectByID2("Top Plane", "PLANE", 0, 0, 0, False, 0, Nothing, 0) Then Exit Sub
    Set f = swModel.FirstFeature
    Do While Not f Is Nothing
        If f.GetTypeName2 = "RefPlane" Then
            n = n + 1
            If n = 2 Then
                If f.Select2(False, 0) Then Exit Sub
                WLog "  Select2 failed on plane " & f.Name
            End If
        End If
        Set f = f.GetNextFeature
    Loop
    Err.Raise vbObjectError + 520, , "Could not select Top Plane (found " & n & " planes)"
End Sub

Private Function EndSketch(ByVal nm As String) As SldWorks.Feature
    Dim sk As SldWorks.Sketch, f As SldWorks.Feature
    Set sk = swSkMgr.ActiveSketch
    If Not sk Is Nothing Then MergeCoincidentPoints sk
    swSkMgr.AddToDB = False
    swSkMgr.DisplayWhenAdded = True
    swSkMgr.InsertSketch True
    Set f = swModel.FeatureByPositionReverse(0)
    If f Is Nothing Then Err.Raise vbObjectError + 522, , "No sketch feature after " & nm
    f.Name = nm
    WLog "  sketch " & nm
    Set EndSketch = f
End Function

' Joins end points that sit on top of each other so every profile is one
' closed contour (no-op if SOLIDWORKS already merged them).
Private Sub MergeCoincidentPoints(sk As SldWorks.Sketch)
    Dim pts As Variant, i As Long, j As Long, n As Long, nStart As Long
    Dim p1 As SldWorks.SketchPoint, p2 As SldWorks.SketchPoint
    Dim found As Boolean, pass As Long, merges As Long
    pts = sk.GetSketchPoints2
    If IsEmpty(pts) Then Exit Sub
    nStart = UBound(pts) + 1
    For pass = 1 To 500
        found = False
        pts = sk.GetSketchPoints2
        n = UBound(pts)
        For i = 0 To n - 1
            Set p1 = pts(i)
            For j = i + 1 To n
                Set p2 = pts(j)
                If Abs(p1.X - p2.X) < 0.0000001 And Abs(p1.Y - p2.Y) < 0.0000001 Then
                    found = True
                    Exit For
                End If
            Next j
            If found Then Exit For
        Next i
        If Not found Then Exit For
        swModel.ClearSelection2 True
        p1.Select4 False, Nothing
        p2.Select4 True, Nothing
        swModel.SketchAddConstraints "sgMERGEPOINTS"
        swModel.ClearSelection2 True
        If UBound(sk.GetSketchPoints2) = n Then
            p1.Select4 False, Nothing
            p2.Select4 True, Nothing
            swModel.SketchAddConstraints "sgCOINCIDENT"
            swModel.ClearSelection2 True
            If UBound(sk.GetSketchPoints2) = n Then
                WLog "  WARNING: duplicate points could not be merged"
                Exit For
            End If
        End If
        merges = merges + 1
    Next pass
    WLog "  points " & nStart & " -> " & (UBound(sk.GetSketchPoints2) + 1) & " (merged " & merges & ")"
End Sub

' Cut-extrude through the plate; tries both-directions first, then each
' single direction, and keeps the first one that actually removes material.
Private Function CutThroughAll(fSk As SldWorks.Feature, ByVal nm As String) As SldWorks.Feature
    Dim fc As SldWorks.Feature, attempt As Long, vBefore As Double, vAfter As Double
    vBefore = BodyVolume()
    For attempt = 1 To 3
        swModel.ClearSelection2 True
        fSk.Select2 False, 0
        Select Case attempt
            Case 1: Set fc = swFeatMgr.FeatureCut4(False, False, False, 1, 1, 0.01, 0.01, False, False, False, False, 0, 0, False, False, False, False, False, True, True, True, True, False, 0, 0, False, False)
            Case 2: Set fc = swFeatMgr.FeatureCut4(True, False, False, 1, 0, 0.01, 0.01, False, False, False, False, 0, 0, False, False, False, False, False, True, True, True, True, False, 0, 0, False, False)
            Case 3: Set fc = swFeatMgr.FeatureCut4(True, True, False, 1, 0, 0.01, 0.01, False, False, False, False, 0, 0, False, False, False, False, False, True, True, True, True, False, 0, 0, False, False)
        End Select
        If Not fc Is Nothing Then
            vAfter = BodyVolume()
            If vBefore < 0 Or vAfter < vBefore - 0.000000000001 Then Exit For
            WLog "  attempt " & attempt & " for " & nm & " removed nothing - undoing"
            swModel.ClearSelection2 True
            fc.Select2 False, 0
            swExt.DeleteSelection2 0
            Set fc = Nothing
        End If
    Next attempt
    If fc Is Nothing Then
        WLog "  ERROR: cut " & nm & " failed"
    Else
        fc.Name = nm
        WLog "  cut " & nm & " ok (attempt " & attempt & "), volume " & Format$(vBefore * 1E+9, "0") & " -> " & Format$(vAfter * 1E+9, "0") & " mm3"
    End If
    Set CutThroughAll = fc
End Function

Private Function BodyVolume() As Double
    Dim v As Variant, st As Long
    On Error Resume Next
    BodyVolume = -1
    v = swExt.GetMassProperties2(1, st, False)
    If IsArray(v) Then BodyVolume = v(3)
End Function

' ---------------------------------------------------------------------
'  Output helpers
' ---------------------------------------------------------------------
Private Sub CloseIfOpen(ByVal fullPath As String)
    Dim docs As Variant, d As Variant, md As SldWorks.ModelDoc2
    docs = swApp.GetDocuments
    If IsEmpty(docs) Then Exit Sub
    For Each d In docs
        Set md = d
        If LCase$(md.GetPathName) = LCase$(fullPath) Then
            WLog "Closing open copy: " & md.GetTitle
            swApp.CloseDoc md.GetTitle
        ElseIf md.GetPathName = "" And Left$(md.GetTitle, 4) = "Part" Then
            WLog "Closing unsaved leftover from an earlier run: " & md.GetTitle
            swApp.CloseDoc md.GetTitle
        End If
    Next
End Sub

Private Function SaveCopy(ByVal suffix As String, ByVal asCopy As Boolean) As Boolean
    Dim errs As Long, warns As Long, opts As Long
    opts = swSaveAsOptions_e.swSaveAsOptions_Silent
    If asCopy Then opts = opts + swSaveAsOptions_e.swSaveAsOptions_Copy
    SaveCopy = swExt.SaveAs(OUT_DIR & PART_NAME & suffix, swSaveAsVersion_e.swSaveAsCurrentVersion, opts, Nothing, errs, warns)
    WLog "Save " & PART_NAME & suffix & ": " & SaveCopy & " (err " & errs & ", warn " & warns & ")"
End Function

Private Sub SetProp(ByVal nm As String, ByVal v As String)
    swExt.CustomPropertyManager("").Add3 nm, swCustomInfoType_e.swCustomInfoText, v, swCustomPropertyAddOption_e.swCustomPropertyReplaceValue
End Sub

Private Sub LogBox(ByVal label As String)
    Dim b As Variant
    b = swPart.GetPartBox(True)
    If IsEmpty(b) Then
        WLog "  " & label & ": no body"
    Else
        WLog "  " & label & " box (mm): X " & F1(b(0)) & " .. " & F1(b(3)) & "   Y " & F1(b(1)) & " .. " & F1(b(4)) & "   Z " & F1(b(2)) & " .. " & F1(b(5))
    End If
End Sub

Private Sub LogTree()
    Dim f As SldWorks.Feature, code As Long, warn As Boolean
    WLog "Feature tree:"
    Set f = swModel.FirstFeature
    Do While Not f Is Nothing
        code = f.GetErrorCode2(warn)
        WLog "   " & f.Name & "  [" & f.GetTypeName2 & "]" & IIf(code <> 0, "  ERROR " & code, "")
        Set f = f.GetNextFeature
    Loop
End Sub

Private Function F1(ByVal meters As Double) As String
    F1 = Format$(meters * 1000#, "0.0")
End Function

Private Sub WLog(ByVal s As String)
    Print #logF, Format$(Now, "hh:nn:ss") & "  " & s
End Sub
