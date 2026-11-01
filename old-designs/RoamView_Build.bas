Attribute VB_Name = "RoamViewBuild"
' =====================================================================
'  RoamView_Build  (SOLIDWORKS VBA macro) - ENGR 490 Capstone, Rev A
'
'  Rebuilds the RoamView chassis from scratch with exact geometry:
'     RoamView_Baseplate.SLDPRT   lower deck: drive, casters, battery, power
'     RoamView_TopDeck.SLDPRT     upper deck: compute, LiDAR, camera, e-stop
'     Components\*.SLDPRT         placeholder envelopes for bought parts
'     RoamView_Chassis.SLDASM     layout assembly of everything above
'  Plates are saved with STEP / STL / PNG copies; everything is logged to
'  RoamView_build_log.txt, including an interference check.
'  Safe to re-run: open copies are closed, rebuilt and overwritten.
'  Change a Const below and re-run to update the whole robot.
'
'  Axes (all parts and the assembly):
'     +X = robot forward, +Y = up, +Z = robot right
'     Top Plane sketches: sketch X = +X (forward), sketch Y = robot left
'     Assembly origin = floor point under the drive-axle centre
'       (ROS base_footprint: x = X, y = -Z, z = Y)
'  All dimensions are millimetres.
' =====================================================================
Option Explicit

Const OUT_DIR As String = "C:\Users\sanom\OneDrive - Concordia University - Canada\ENGR 490  - Capstone\RoamView\old-designs\"
Const COMP_DIR As String = OUT_DIR & "Components\"
Const ASM_NAME As String = "RoamView_Chassis"

' --- Shared footprint --------------------------------------------------
Const PLATE_L As Double = 320      ' length along X (forward)
Const PLATE_W As Double = 260      ' width (robot left-right)
Const CORNER_R As Double = 20      ' corner radius, concentric with deck posts
Const POST_X As Double = 140       ' 2020 deck posts at (+-POST_X, +-POST_Y)
Const POST_Y As Double = 110
Const POST_H As Double = 120       ' post length = gap between decks

' --- Baseplate (lower deck) ---------------------------------------------
Const BASE_T As Double = 6         ' 3/16 in = 4.76 saves ~0.25 kg
Const WELL_HALF As Double = 62     ' wheel wells span X = -62 .. +62
Const WELL_DEPTH As Double = 42    ' cut in from each side edge
Const WELL_R As Double = 6         ' inner corner radius

' --- Top deck ----------------------------------------------------------
Const DECK_T As Double = 3         ' 1/8 in aluminium

' --- Drivetrain and height stack (from the floor) ------------------------
Const WHEEL_D As Double = 100      ' drive wheel diameter
Const WHEEL_W As Double = 26       ' drive wheel width
Const TRACK As Double = 220        ' wheel centre to wheel centre
Const MOTOR_FACE As Double = 84    ' gearbox face distance from centreline
Const AXLE_Y As Double = WHEEL_D / 2
Const BASE_Y As Double = 72        ' underside of baseplate (motor top + bracket)
Const DECK_Y As Double = BASE_Y + BASE_T + POST_H
Const RISER_H As Double = 60       ' LiDAR riser height above the top deck

' --- Clearance hole sizes ----------------------------------------------
Const M25 As Double = 2.7
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
Dim curPart As String
Dim curDir As String

Sub main()
    logF = FreeFile
    Open OUT_DIR & "RoamView_build_log.txt" For Output As #logF
    On Error GoTo EH

    curStep = "connect"
    Set swApp = Application.SldWorks
    WLog "RoamView chassis build - SOLIDWORKS " & swApp.RevisionNumber
    CloseIfOpen OUT_DIR & ASM_NAME & ".SLDASM"

    BuildBaseplate
    BuildTopDeck
    BuildComponents
    BuildAssembly

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

' =====================================================================
'  Lower deck: 320 x 260 x 6 plate with wheel wells for 100 mm wheels
' =====================================================================
Private Sub BuildBaseplate()
    Dim fSk As SldWorks.Feature
    Dim sv As Variant, xv As Variant, cx As Variant, dx As Variant, yv As Variant
    Dim i As Long

    StartPart "RoamView_Baseplate", OUT_DIR

    curStep = "baseplate outline"
    StartSketch
    BaseOutline
    Set fSk = EndSketch("SK_Plate")
    ExtrudePlate fSk, BASE_T, "Plate"

    ' Drive-motor bracket slots under the plate on the axle line.
    ' M3, slotted along Y so the wheel-to-well clearance can be set.
    curStep = "motor mounts"
    StartSketch
    For Each sv In Array(1, -1)
        For Each xv In Array(-12.5, 12.5)
            Slot xv, sv * 64, xv, sv * 76, M3
        Next
    Next
    Set fSk = EndSketch("SK_MotorMounts")
    CutThroughAll fSk, "Cut_MotorMounts_M3"

    ' Casters front (+X, anti-tip) and rear (-X, load-bearing), M4 slots
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

    ' 4S2P battery bay over the axle: 25 mm strap slots + tray holes
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

    ' Cable pass-throughs front and rear of the battery
    curStep = "cable pass-throughs"
    StartSketch
    Slot 58, -20, 58, 20, 12
    Slot -58, -20, -58, 20, 12
    Set fSk = EndSketch("SK_CablePassThru")
    CutThroughAll fSk, "Cut_CablePassThru"

    ' Electronics grid, M3 on 20 mm pitch, front and rear zones
    curStep = "electronics grid"
    StartSketch
    For Each xv In Array(-100, -80, 80, 100)
        For i = -5 To 5
            Hole xv, i * 20, M3
        Next
    Next
    Set fSk = EndSketch("SK_ElecGrid_M3")
    CutThroughAll fSk, "Cut_ElecGrid_M3"

    ' Top-deck posts: 2020 extrusion, M5 into the centre bore
    curStep = "base deck posts"
    StartSketch
    PostHoles
    Set fSk = EndSketch("SK_DeckPosts_M5")
    CutThroughAll fSk, "Cut_DeckPosts_M5"

    ' Downward cliff-sensor windows at all four corners
    curStep = "cliff sensors"
    StartSketch
    For Each xv In Array(145, -145)
        For Each yv In Array(88, -88)
            Hole xv, yv, 12
        Next
    Next
    Set fSk = EndSketch("SK_CliffSensors")
    CutThroughAll fSk, "Cut_CliffSensorWindows"

    ' Dock charging contacts at the front, 2 x M4 per contact
    curStep = "charging contacts"
    StartSketch
    For Each yv In Array(40, 60, -40, -60)
        Hole 150, yv, M4
    Next
    Set fSk = EndSketch("SK_ChargeContacts")
    CutThroughAll fSk, "Cut_ChargeContacts_M4"

    FinishPlate "RoamView lower chassis deck (baseplate)", "RV-CH-001", "6061-T6 aluminium plate, 6 mm", _
                "Motor and caster patterns are adjustment slots - confirm against purchased parts"
End Sub

' =====================================================================
'  Upper deck: 320 x 260 x 3 plate on the same four posts
' =====================================================================
Private Sub BuildTopDeck()
    Dim fSk As SldWorks.Feature
    Dim xv As Variant, yv As Variant

    StartPart "RoamView_TopDeck", OUT_DIR

    curStep = "top deck outline"
    StartSketch
    RoundedRect PLATE_L, PLATE_W, CORNER_R
    Set fSk = EndSketch("SK_Deck")
    ExtrudePlate fSk, DECK_T, "Deck"

    curStep = "top deck posts"
    StartSketch
    PostHoles
    Set fSk = EndSketch("SK_DeckPosts_M5")
    CutThroughAll fSk, "Cut_DeckPosts_M5"

    ' D500 LiDAR riser directly above the drive axle (4 x M3 on 40 x 40)
    ' plus a 20 mm hole for the LiDAR cable
    curStep = "lidar mount"
    StartSketch
    For Each xv In Array(20, -20)
        For Each yv In Array(20, -20)
            Hole xv, yv, M3
        Next
    Next
    Hole 0, 0, 20
    Set fSk = EndSketch("SK_LidarMount")
    CutThroughAll fSk, "Cut_LidarMount"

    ' Raspberry Pi 5 standoffs, M2.5 on 58 x 49, rear-left
    curStep = "pi5 standoffs"
    StartSketch
    For Each xv In Array(-119, -61)
        For Each yv In Array(40.5, 89.5)
            Hole xv, yv, M25
        Next
    Next
    Set fSk = EndSketch("SK_Pi5_M2p5")
    CutThroughAll fSk, "Cut_Pi5_M2p5"

    ' M3 grid, 20 mm pitch, rear-right: ESP32 carrier, USB hub, compute buck
    curStep = "deck grid"
    StartSketch
    For Each xv In Array(-120, -100, -80, -60, -40)
        For Each yv In Array(-40, -60, -80, -100)
            Hole xv, yv, M3
        Next
    Next
    Set fSk = EndSketch("SK_DeckGrid_M3")
    CutThroughAll fSk, "Cut_DeckGrid_M3"

    ' Emergency stop, 22 mm panel-mount mushroom button, rear centre
    curStep = "e-stop"
    StartSketch
    Hole -130, 0, 22.5
    Set fSk = EndSketch("SK_EStop_22mm")
    CutThroughAll fSk, "Cut_EStop_22mm"

    ' OAK-D Pro W bracket at the front edge, kept below the LiDAR plane
    curStep = "camera mount"
    StartSketch
    For Each xv In Array(120, 140)
        For Each yv In Array(30, -30)
            Hole xv, yv, M4
        Next
    Next
    Set fSk = EndSketch("SK_CameraMount_M4")
    CutThroughAll fSk, "Cut_CameraMount_M4"

    ' Recording indicator (Law 25) front-left, status light front-right
    curStep = "indicator lights"
    StartSketch
    Hole 130, 75, 16
    Hole 130, -75, 16
    Set fSk = EndSketch("SK_IndicatorLights")
    CutThroughAll fSk, "Cut_IndicatorLights_16mm"

    ' Cable pass-throughs down to the lower deck
    curStep = "deck pass-throughs"
    StartSketch
    Slot -10, 60, 30, 60, 12
    Slot -10, -60, 30, -60, 12
    Set fSk = EndSketch("SK_DeckPassThru")
    CutThroughAll fSk, "Cut_DeckPassThru"

    FinishPlate "RoamView upper chassis deck (compute + sensors)", "RV-CH-002", "6061-T6 aluminium plate, 3 mm", _
                "LiDAR riser sits over the drive axle; 22 mm e-stop hole at rear centre"
End Sub

' =====================================================================
'  Placeholder envelopes for bought parts (replace with vendor CAD)
'  Each part origin is the point used to place it in the assembly.
' =====================================================================
Private Sub BuildComponents()
    Dim fSk As SldWorks.Feature, sd As Variant, nm As String

    On Error Resume Next
    MkDir COMP_DIR
    On Error GoTo 0

    ' Drive wheel: origin = wheel centre, axis along Z
    StartPart "RoamView_DriveWheel", COMP_DIR
    RevolveProfile "Wheel", True, Array(0, -13, 12, -13, 12, -8, 38, -8, 38, -13, 50, -13, 50, 13, 38, 13, 38, 8, 12, 8, 12, 13, 0, 13)
    FinishComponent "Drive wheel 100 x 26 mm", 0.12, 0.12, 0.12

    ' Gearmotor + L-bracket: origin = gearbox face on the axle.
    ' Left motor body runs toward +Z (inboard), shaft toward -Z; right is mirrored.
    For Each sd In Array(1, -1)
        If sd = 1 Then nm = "RoamView_DriveMotor_L" Else nm = "RoamView_DriveMotor_R"
        StartPart nm, COMP_DIR
        RevolveProfile "Gearmotor", True, Array(0, -18 * sd, 3, -18 * sd, 3, 0, 18.5, 0, 18.5, 70 * sd, 0, 70 * sd)
        BoxRight "Bracket_Face", -3 * sd, -22.5, 0, BASE_Y - AXLE_Y, 45
        BoxRight "Bracket_Top", -3 * sd, BASE_Y - AXLE_Y - 3, 30 * sd, BASE_Y - AXLE_Y, 45
        FinishComponent "37 mm gearmotor with encoder + L-bracket", 0.55, 0.55, 0.6
    Next

    ' Swivel caster: origin = caster wheel centre (25 mm above floor)
    StartPart "RoamView_Caster", COMP_DIR
    RevolveProfile "CasterWheel", True, Array(0, -10, 25, -10, 25, 10, 0, 10)
    BoxFront "Fork", -15, 20, 15, BASE_Y - 25 - 3, 30
    BoxFront "TopPlate", -25, BASE_Y - 25 - 3, 25, BASE_Y - 25, 40
    FinishComponent "2 in swivel caster, 72 mm to mount face", 0.3, 0.3, 0.3

    ' 4S2P battery: origin = bottom centre
    StartPart "RoamView_Battery4S2P", COMP_DIR
    BoxFront "Pack", -45, 0, 45, 42, 75
    FinishComponent "4S2P 18650 pack with BMS, 90 x 75 x 42", 0.1, 0.35, 0.8

    ' 2020 deck post: origin = bottom centre
    StartPart "RoamView_Post2020", COMP_DIR
    curStep = "post profile"
    StartSketch
    Post2020Profile
    Hole 0, 0, 4.2
    Set fSk = EndSketch("SK_Profile")
    ExtrudePlate fSk, POST_H, "Extrusion"
    FinishComponent "2020 aluminium extrusion, " & POST_H & " mm deck post", 0.75, 0.75, 0.78

    ' Printed LiDAR riser: origin = bottom centre
    StartPart "RoamView_LidarRiser", COMP_DIR
    BoxFront "Riser", -25, 0, 25, RISER_H, 50
    FinishComponent "3D-printed LiDAR riser 50 x 50 x " & RISER_H, 0.95, 0.6, 0.1

    ' Waveshare D500 LiDAR: origin = bottom centre
    StartPart "RoamView_LidarD500", COMP_DIR
    BoxFront "Base", -27, 0, 27, 20, 46
    RevolveProfile "Head", False, Array(0, 20, 19, 20, 19, 35, 0, 35)
    FinishComponent "Waveshare D500 LiDAR envelope", 0.15, 0.15, 0.18

    ' OAK-D Pro W on an L-bracket: origin = bracket base centre on the deck
    StartPart "RoamView_OakDProW", COMP_DIR
    BoxFront "Bracket_Base", -23, 0, 17, 3, 80
    BoxFront "Bracket_Upright", -14, 0, -11, 40, 60
    BoxFront "Camera", -11, 10, 11, 40, 97
    FinishComponent "Luxonis OAK-D Pro W 97 x 30 x 22 on bracket", 0.08, 0.08, 0.08

    ' 22 mm mushroom e-stop: origin = top face of the deck
    StartPart "RoamView_EStop", COMP_DIR
    RevolveProfile "EStop", False, Array(0, -45, 15, -45, 15, -DECK_T, 11, -DECK_T, 11, 0, 15, 0, 15, 18, 20, 18, 20, 30, 0, 30)
    FinishComponent "22 mm mushroom e-stop with NC contact block", 0.85, 0.1, 0.1

    ' Raspberry Pi 5 with active cooler: origin = board underside centre
    StartPart "RoamView_Pi5", COMP_DIR
    BoxFront "Board", -42.5, 0, 42.5, 1.6, 56
    BoxFront "Cooler", -25, 1.6, 15, 11.6, 40
    BoxFront "Ports", 25, 1.6, 42.5, 17.6, 53
    FinishComponent "Raspberry Pi 5 + active cooler", 0.1, 0.55, 0.25
End Sub

' =====================================================================
'  Layout assembly: every component placed by exact coordinates, fixed
' =====================================================================
Private Sub BuildAssembly()
    Dim tmpl As String, asm As SldWorks.ModelDoc2, swAssy As SldWorks.AssemblyDoc
    Dim xv As Variant, zv As Variant, vComps As Variant, vc As Variant, c As SldWorks.Component2

    curStep = "assembly: new"
    WLog "=== " & ASM_NAME & " (assembly) ==="
    tmpl = swApp.GetUserPreferenceStringValue(swUserPreferenceStringValue_e.swDefaultTemplateAssembly)
    Set asm = swApp.NewDocument(tmpl, 0, 0, 0)
    If asm Is Nothing Then Err.Raise vbObjectError + 530, , "Could not create an assembly"
    Set swAssy = asm
    swApp.ActivateDoc3 asm.GetTitle, False, swRebuildOnActivation_e.swDontRebuildActiveDoc, 0
    asm.Extension.SetUserPreferenceInteger swUserPreferenceIntegerValue_e.swUnitSystem, 0, swUnitSystem_e.swUnitSystem_MMGS

    Place swAssy, OUT_DIR & "RoamView_Baseplate.SLDPRT", 0, BASE_Y, 0
    Place swAssy, OUT_DIR & "RoamView_TopDeck.SLDPRT", 0, DECK_Y, 0
    For Each xv In Array(POST_X, -POST_X)
        For Each zv In Array(POST_Y, -POST_Y)
            Place swAssy, COMP_DIR & "RoamView_Post2020.SLDPRT", xv, BASE_Y + BASE_T, zv
        Next
    Next
    Place swAssy, COMP_DIR & "RoamView_DriveWheel.SLDPRT", 0, AXLE_Y, -TRACK / 2
    Place swAssy, COMP_DIR & "RoamView_DriveWheel.SLDPRT", 0, AXLE_Y, TRACK / 2
    Place swAssy, COMP_DIR & "RoamView_DriveMotor_L.SLDPRT", 0, AXLE_Y, -MOTOR_FACE
    Place swAssy, COMP_DIR & "RoamView_DriveMotor_R.SLDPRT", 0, AXLE_Y, MOTOR_FACE
    Place swAssy, COMP_DIR & "RoamView_Caster.SLDPRT", 130, 25, 0
    Place swAssy, COMP_DIR & "RoamView_Caster.SLDPRT", -130, 25, 0
    Place swAssy, COMP_DIR & "RoamView_Battery4S2P.SLDPRT", 0, BASE_Y + BASE_T, 0
    Place swAssy, COMP_DIR & "RoamView_LidarRiser.SLDPRT", 0, DECK_Y + DECK_T, 0
    Place swAssy, COMP_DIR & "RoamView_LidarD500.SLDPRT", 0, DECK_Y + DECK_T + RISER_H, 0
    Place swAssy, COMP_DIR & "RoamView_OakDProW.SLDPRT", 138, DECK_Y + DECK_T, 0
    Place swAssy, COMP_DIR & "RoamView_EStop.SLDPRT", -130, DECK_Y + DECK_T, 0
    Place swAssy, COMP_DIR & "RoamView_Pi5.SLDPRT", -80, DECK_Y + DECK_T + 10, -65

    curStep = "assembly: fix components"
    asm.EditRebuild3
    asm.ClearSelection2 True
    vComps = swAssy.GetComponents(True)
    For Each vc In vComps
        Set c = vc
        c.Select4 True, Nothing, False
    Next
    swAssy.FixComponent
    asm.ClearSelection2 True
    asm.EditRebuild3
    WLog "  components: " & (UBound(vComps) + 1)

    curStep = "assembly: interference check"
    CheckInterference swAssy

    curStep = "assembly: save"
    Set swModel = asm
    Set swExt = asm.Extension
    curPart = ASM_NAME: curDir = OUT_DIR
    asm.ShowNamedView2 "*Isometric", 7
    asm.ViewZoomtofit2
    SaveCopy ".SLDASM", False
    SaveCopy "_iso.png", True
    asm.ShowNamedView2 "*Trimetric", 8
    asm.ViewZoomtofit2
    SaveCopy "_trimetric.png", True
    asm.ShowNamedView2 "*Front", 1
    asm.ViewZoomtofit2
    SaveCopy "_side.png", True
    asm.ShowNamedView2 "*Top", 5
    asm.ViewZoomtofit2
    SaveCopy "_top.png", True
    asm.ShowNamedView2 "*Trimetric", 8
    asm.ViewZoomtofit2
End Sub

Private Function Place(swAssy As SldWorks.AssemblyDoc, ByVal path As String, _
                       ByVal x As Double, ByVal y As Double, ByVal z As Double) As SldWorks.Component2
    Dim c As SldWorks.Component2, mu As SldWorks.MathUtility, t As SldWorks.MathTransform
    Dim d(15) As Double, v As Variant
    curStep = "assembly: add " & Mid$(path, InStrRev(path, "\") + 1)
    Set c = swAssy.AddComponent5(path, 0, "", False, "", Mt(x), Mt(y), Mt(z))
    If c Is Nothing Then Err.Raise vbObjectError + 531, , "AddComponent failed for " & path
    Set mu = swApp.GetMathUtility
    d(0) = 1: d(4) = 1: d(8) = 1
    d(9) = Mt(x): d(10) = Mt(y): d(11) = Mt(z)
    d(12) = 1
    v = d
    Set t = mu.CreateTransform(v)
    c.Transform2 = t
    WLog "  placed " & c.Name2 & " at X " & x & "  Y " & y & "  Z " & z
    Set Place = c
End Function

Private Sub CheckInterference(swAssy As SldWorks.AssemblyDoc)
    Dim idm As SldWorks.InterferenceDetectionMgr, n As Long, i As Long
    Dim vI As Variant, itf As SldWorks.IInterference, comps As Variant, s As String, j As Long
    Set idm = swAssy.InterferenceDetectionManager
    idm.TreatCoincidenceAsInterference = False
    idm.TreatSubAssembliesAsComponents = True
    idm.IncludeMultibodyPartInterferences = False
    n = idm.GetInterferenceCount
    WLog "Interference check: " & n & " interference(s)"
    If n > 0 Then
        vI = idm.GetInterferences
        For i = 0 To UBound(vI)
            Set itf = vI(i)
            comps = itf.Components
            s = ""
            For j = 0 To UBound(comps)
                s = s & IIf(j > 0, " <-> ", "") & comps(j).Name2
            Next j
            WLog "   " & s & "   " & Format$(itf.Volume * 1E+9, "0.0") & " mm3"
        Next i
    End If
    idm.Done
End Sub

' ---------------------------------------------------------------------
'  Outlines (closed loops drawn counter-clockwise, lines + tangent arcs)
' ---------------------------------------------------------------------
Private Sub BaseOutline()
    Dim hx As Double, hy As Double, R As Double
    Dim wx As Double, wy As Double, rw As Double
    hx = PLATE_L / 2: hy = PLATE_W / 2: R = CORNER_R
    wx = WELL_HALF: wy = hy - WELL_DEPTH: rw = WELL_R

    Ln hx, -hy + R, hx, hy - R                          ' front edge
    Ar hx - R, hy - R, hx, hy - R, hx - R, hy, 1        ' front-left corner
    Ln hx - R, hy, wx, hy                               ' left edge (front)
    Ln wx, hy, wx, wy + rw                              ' left wheel well
    Ar wx - rw, wy + rw, wx, wy + rw, wx - rw, wy, -1
    Ln wx - rw, wy, -wx + rw, wy
    Ar -wx + rw, wy + rw, -wx + rw, wy, -wx, wy + rw, -1
    Ln -wx, wy + rw, -wx, hy
    Ln -wx, hy, -hx + R, hy                             ' left edge (rear)
    Ar -hx + R, hy - R, -hx + R, hy, -hx, hy - R, 1     ' rear-left corner
    Ln -hx, hy - R, -hx, -hy + R                        ' rear edge
    Ar -hx + R, -hy + R, -hx, -hy + R, -hx + R, -hy, 1  ' rear-right corner
    Ln -hx + R, -hy, -wx, -hy                           ' right edge (rear)
    Ln -wx, -hy, -wx, -wy - rw                          ' right wheel well
    Ar -wx + rw, -wy - rw, -wx, -wy - rw, -wx + rw, -wy, -1
    Ln -wx + rw, -wy, wx - rw, -wy
    Ar wx - rw, -wy - rw, wx - rw, -wy, wx, -wy - rw, -1
    Ln wx, -wy - rw, wx, -hy
    Ln wx, -hy, hx - R, -hy                             ' right edge (front)
    Ar hx - R, -hy + R, hx - R, -hy, hx, -hy + R, 1     ' front-right corner
End Sub

Private Sub RoundedRect(ByVal lx As Double, ByVal ly As Double, ByVal r As Double)
    Dim hx As Double, hy As Double
    hx = lx / 2: hy = ly / 2
    Ln hx, -hy + r, hx, hy - r
    Ar hx - r, hy - r, hx, hy - r, hx - r, hy, 1
    Ln hx - r, hy, -hx + r, hy
    Ar -hx + r, hy - r, -hx + r, hy, -hx, hy - r, 1
    Ln -hx, hy - r, -hx, -hy + r
    Ar -hx + r, -hy + r, -hx, -hy + r, -hx + r, -hy, 1
    Ln -hx + r, -hy, hx - r, -hy
    Ar hx - r, -hy + r, hx - r, -hy, hx, -hy + r, 1
End Sub

' 20 x 20 profile with a 6.2 mm slot on each face (simplified T-slot)
Private Sub Post2020Profile()
    Dim p As Variant, i As Long, n As Long
    p = Array(10, -10, 10, -3.1, 4, -3.1, 4, 3.1, 10, 3.1, 10, 10, 3.1, 10, 3.1, 4, -3.1, 4, -3.1, 10, _
              -10, 10, -10, 3.1, -4, 3.1, -4, -3.1, -10, -3.1, -10, -10, -3.1, -10, -3.1, -4, 3.1, -4, 3.1, -10)
    n = (UBound(p) + 1) \ 2
    For i = 0 To n - 1
        Ln p(2 * i), p(2 * i + 1), p(2 * ((i + 1) Mod n)), p(2 * ((i + 1) Mod n) + 1)
    Next
End Sub

Private Sub PostHoles()
    Dim xv As Variant, yv As Variant
    For Each xv In Array(POST_X, -POST_X)
        For Each yv In Array(POST_Y, -POST_Y)
            Hole xv, yv, M5
        Next
    Next
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

Private Sub Rect(ByVal x1 As Double, ByVal y1 As Double, ByVal x2 As Double, ByVal y2 As Double)
    Ln x1, y1, x2, y1
    Ln x2, y1, x2, y2
    Ln x2, y2, x1, y2
    Ln x1, y2, x1, y1
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

' ---------------------------------------------------------------------
'  Solid helpers for placeholder parts
' ---------------------------------------------------------------------
' Box from a Front Plane rectangle (X,Y), extruded symmetrically along Z
Private Sub BoxFront(ByVal nm As String, ByVal x1 As Double, ByVal y1 As Double, _
                     ByVal x2 As Double, ByVal y2 As Double, ByVal depthZ As Double)
    Dim fSk As SldWorks.Feature
    curStep = curPart & ": " & nm
    StartSketchOn "Front Plane", 1
    Rect x1, y1, x2, y2
    Set fSk = EndSketch("SK_" & nm)
    ExtrudeMid fSk, depthZ, nm
End Sub

' Box from a Right Plane rectangle (model Z,Y), extruded symmetrically along X.
' Right Plane sketch X runs along model -Z.
Private Sub BoxRight(ByVal nm As String, ByVal z1 As Double, ByVal y1 As Double, _
                     ByVal z2 As Double, ByVal y2 As Double, ByVal depthX As Double)
    Dim fSk As SldWorks.Feature
    curStep = curPart & ": " & nm
    StartSketchOn "Right Plane", 3
    Rect -z1, y1, -z2, y2
    Set fSk = EndSketch("SK_" & nm)
    ExtrudeMid fSk, depthX, nm
End Sub

' Revolve a closed (radius, axial) polygon 360 deg about model Z (aboutZ)
' or model Y. pts = Array(r1, a1, r2, a2, ...), last point joins the first.
Private Sub RevolveProfile(ByVal nm As String, ByVal aboutZ As Boolean, pts As Variant)
    Dim n As Long, i As Long, k As Long, s As Double, amin As Double, amax As Double
    Dim fSk As SldWorks.Feature, f As SldWorks.Feature
    curStep = curPart & ": " & nm
    If aboutZ Then
        StartSketchOn "Top Plane", 2
        s = -1                         ' Top Plane sketch Y = model -Z
    Else
        StartSketchOn "Front Plane", 1
        s = 1                          ' Front Plane sketch Y = model Y
    End If
    n = (UBound(pts) + 1) \ 2
    amin = 1E+30: amax = -1E+30
    For i = 0 To n - 1
        k = (i + 1) Mod n
        Ln pts(2 * i), s * pts(2 * i + 1), pts(2 * k), s * pts(2 * k + 1)
        If pts(2 * i + 1) < amin Then amin = pts(2 * i + 1)
        If pts(2 * i + 1) > amax Then amax = pts(2 * i + 1)
    Next
    swSkMgr.CreateCenterLine 0#, Mt(s * (amin - 5)), 0#, 0#, Mt(s * (amax + 5)), 0#
    Set fSk = EndSketch("SK_" & nm)
    swModel.ClearSelection2 True
    fSk.Select2 False, 0
    Set f = swFeatMgr.FeatureRevolve2(True, True, False, False, False, False, 0, 0, 6.28318530717959, 0, _
                                      False, False, 0, 0, 0, 0, 0, True, True, True)
    If f Is Nothing Then Err.Raise vbObjectError + 517, , nm & " revolve failed"
    f.Name = nm
    LogBox nm
End Sub

' ---------------------------------------------------------------------
'  Part / feature helpers
' ---------------------------------------------------------------------
Private Sub StartPart(ByVal nm As String, ByVal folder As String)
    Dim tmpl As String
    curPart = nm: curDir = folder
    curStep = nm & ": new part"
    WLog "=== " & nm & " ==="
    CloseIfOpen folder & nm & ".SLDPRT"
    tmpl = swApp.GetUserPreferenceStringValue(swUserPreferenceStringValue_e.swDefaultTemplatePart)
    Set swModel = swApp.NewDocument(tmpl, 0, 0, 0)
    If swModel Is Nothing Then Set swModel = swApp.NewPart
    If swModel Is Nothing Then Err.Raise vbObjectError + 513, , "Could not create a new part"
    Set swPart = swModel
    Set swExt = swModel.Extension
    Set swSkMgr = swModel.SketchManager
    Set swFeatMgr = swModel.FeatureManager
    swApp.ActivateDoc3 swModel.GetTitle, False, swRebuildOnActivation_e.swDontRebuildActiveDoc, 0
    swExt.SetUserPreferenceInteger swUserPreferenceIntegerValue_e.swUnitSystem, 0, swUnitSystem_e.swUnitSystem_MMGS
End Sub

Private Sub StartSketch()
    StartSketchOn "Top Plane", 2
End Sub

Private Sub StartSketchOn(ByVal planeName As String, ByVal idx As Long)
    swModel.ClearSelection2 True
    SelectPlane planeName, idx
    swSkMgr.InsertSketch True
    If swSkMgr.ActiveSketch Is Nothing Then Err.Raise vbObjectError + 521, , "Sketch did not start on " & planeName
    swSkMgr.AddToDB = True
    swSkMgr.DisplayWhenAdded = False
End Sub

' Default planes are Front / Top / Right = 1st / 2nd / 3rd reference plane in
' every standard template; fall back to that if names are localized.
Private Sub SelectPlane(ByVal planeName As String, ByVal idx As Long)
    Dim f As SldWorks.Feature, n As Long
    If swExt.SelectByID2(planeName, "PLANE", 0, 0, 0, False, 0, Nothing, 0) Then Exit Sub
    Set f = swModel.FirstFeature
    Do While Not f Is Nothing
        If f.GetTypeName2 = "RefPlane" Then
            n = n + 1
            If n = idx Then
                If f.Select2(False, 0) Then Exit Sub
            End If
        End If
        Set f = f.GetNextFeature
    Loop
    Err.Raise vbObjectError + 520, , "Could not select " & planeName
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
    Set EndSketch = f
End Function

' Joins end points that sit on top of each other so every profile is one
' closed contour (no-op when SOLIDWORKS has already merged them).
Private Sub MergeCoincidentPoints(sk As SldWorks.Sketch)
    Dim pts As Variant, i As Long, j As Long, n As Long
    Dim p1 As SldWorks.SketchPoint, p2 As SldWorks.SketchPoint
    Dim found As Boolean, pass As Long, merges As Long
    pts = sk.GetSketchPoints2
    If IsEmpty(pts) Then Exit Sub
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
            WLog "  WARNING: duplicate points could not be merged"
            Exit For
        End If
        merges = merges + 1
    Next pass
    If merges > 0 Then WLog "  merged " & merges & " duplicate points"
End Sub

Private Sub ExtrudePlate(fSk As SldWorks.Feature, ByVal t As Double, ByVal nm As String)
    Dim f As SldWorks.Feature
    curStep = curPart & ": extrude " & nm
    swModel.ClearSelection2 True
    fSk.Select2 False, 0
    Set f = swFeatMgr.FeatureExtrusion2(True, False, False, 0, 0, Mt(t), 0, False, False, False, False, 0, 0, _
                                        False, False, False, False, True, True, True, 0, 0, False)
    If f Is Nothing Then Err.Raise vbObjectError + 514, , nm & " extrusion failed (profile not closed?)"
    f.Name = nm
    LogBox nm
End Sub

Private Sub ExtrudeMid(fSk As SldWorks.Feature, ByVal depth As Double, ByVal nm As String)
    Dim f As SldWorks.Feature
    swModel.ClearSelection2 True
    fSk.Select2 False, 0
    Set f = swFeatMgr.FeatureExtrusion2(True, False, False, swEndConditions_e.swEndCondMidPlane, 0, Mt(depth), 0, _
                                        False, False, False, False, 0, 0, False, False, False, False, True, True, True, 0, 0, False)
    If f Is Nothing Then Err.Raise vbObjectError + 516, , nm & " mid-plane extrusion failed"
    f.Name = nm
    LogBox nm
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
    If fc Is Nothing Then Err.Raise vbObjectError + 515, , "Cut " & nm & " failed"
    fc.Name = nm
    WLog "  " & nm & ": volume " & Format$(vBefore * 1E+9, "0") & " -> " & Format$(vAfter * 1E+9, "0") & " mm3"
    Set CutThroughAll = fc
End Function

Private Function BodyVolume() As Double
    Dim v As Variant, st As Long
    On Error Resume Next
    BodyVolume = -1
    v = swExt.GetMassProperties2(1, st, False)
    If IsArray(v) Then BodyVolume = v(3)
End Function

Private Sub FinishPlate(ByVal desc As String, ByVal partNo As String, ByVal stock As String, ByVal notes As String)
    Dim db As String, mp As Variant, st As Long
    curStep = curPart & ": material"
    swPart.SetMaterialPropertyName2 "Default", "SOLIDWORKS Materials", "6061-T6 (SS)"
    WLog "Material: " & swPart.GetMaterialPropertyName2("Default", db) & " [" & db & "]"

    curStep = curPart & ": properties"
    SetProp "Description", desc
    SetProp "PartNo", partNo
    SetProp "Revision", "A"
    SetProp "Project", "ENGR 490 RoamView"
    SetProp "Stock", stock
    SetProp "Notes", notes

    curStep = curPart & ": checks"
    swModel.ForceRebuild3 False
    mp = swExt.GetMassProperties2(1, st, False)
    If IsArray(mp) Then
        WLog "Mass: " & Format$(mp(5), "0.000") & " kg   Volume: " & Format$(mp(3) * 1000000#, "0.0") & " cm3"
        WLog "Centre of mass (mm): X " & F1(mp(0)) & "  Y " & F1(mp(1)) & "  Z " & F1(mp(2))
    End If
    LogBox "Final"
    LogTree

    curStep = curPart & ": save"
    swModel.ClearSelection2 True
    swModel.ShowNamedView2 "*Isometric", 7
    swModel.ViewZoomtofit2
    SaveCopy ".SLDPRT", False
    SaveCopy ".STEP", True
    SaveCopy ".STL", True
    SaveCopy "_iso.png", True
    swModel.ShowNamedView2 "*Top", 5
    swModel.ViewZoomtofit2
    SaveCopy "_top.png", True
    swModel.ShowNamedView2 "*Isometric", 7
    swModel.ViewZoomtofit2
End Sub

Private Sub FinishComponent(ByVal desc As String, ByVal r As Double, ByVal g As Double, ByVal b As Double)
    Dim v As Variant
    curStep = curPart & ": finish"
    v = swModel.MaterialPropertyValues
    If IsArray(v) Then
        v(0) = r: v(1) = g: v(2) = b
        swModel.MaterialPropertyValues = v
    End If
    SetProp "Description", desc
    SetProp "Project", "ENGR 490 RoamView"
    SetProp "Notes", "Placeholder envelope for layout only - replace with vendor CAD"
    swModel.ForceRebuild3 False
    swModel.ClearSelection2 True
    swModel.ShowNamedView2 "*Isometric", 7
    swModel.ViewZoomtofit2
    SaveCopy ".SLDPRT", False
End Sub

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
        ElseIf md.GetPathName = "" And (Left$(md.GetTitle, 4) = "Part" Or Left$(md.GetTitle, 5) = "Assem") Then
            WLog "Closing unsaved leftover from an earlier run: " & md.GetTitle
            swApp.CloseDoc md.GetTitle
        End If
    Next
End Sub

Private Function SaveCopy(ByVal suffix As String, ByVal asCopy As Boolean) As Boolean
    Dim errs As Long, warns As Long, opts As Long, ok As Boolean
    opts = swSaveAsOptions_e.swSaveAsOptions_Silent
    If asCopy Then opts = opts + swSaveAsOptions_e.swSaveAsOptions_Copy
    ok = swExt.SaveAs(curDir & curPart & suffix, swSaveAsVersion_e.swSaveAsCurrentVersion, opts, Nothing, errs, warns)
    WLog "Save " & curPart & suffix & ": " & IIf(errs = 0, "ok", "ERROR " & errs) & IIf(warns <> 0, " (warning " & warns & ")", "")
    SaveCopy = (errs = 0)
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
    Dim f As SldWorks.Feature, code As Long, warn As Boolean, tn As String
    WLog "Feature tree:"
    Set f = swModel.FirstFeature
    Do While Not f Is Nothing
        tn = f.GetTypeName2
        If InStr(tn, "Folder") = 0 And tn <> "DetailCabinet" Then
            code = f.GetErrorCode2(warn)
            WLog "   " & f.Name & "  [" & tn & "]" & IIf(code <> 0, "  ERROR " & code, "")
        End If
        Set f = f.GetNextFeature
    Loop
End Sub

Private Function F1(ByVal meters As Double) As String
    F1 = Format$(meters * 1000#, "0.0")
End Function

Private Sub WLog(ByVal s As String)
    Print #logF, Format$(Now, "hh:nn:ss") & "  " & s
End Sub
