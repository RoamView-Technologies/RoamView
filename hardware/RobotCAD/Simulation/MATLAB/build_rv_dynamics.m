function mdl = build_rv_dynamics(P, look, scene)
%BUILD_RV_DYNAMICS  Generate rv_dynamics.slx, the Simscape Multibody model of RoamView.
%   build_rv_dynamics() regenerates the model from rv_params(). Run it again
%   after any change to the globals, masses or this file. The model reads its
%   per-run settings from two workspace variables: RV (see rv_tunables.m) and
%   RV_ref, a [t v yaw_rate] table of speed commands.
%
%   Bodies (ROS frame: x forward, y left, z up; origin on the floor under the axle)
%     chassis      6-DOF to the world; mass, COM and inertia from SolidWorks minus
%                  the bodies below
%     drive wheels revolute about y, torque from the motor model; spherical
%                  contact geometry of the tyre radius (same profile as the tyre
%                  for floor and threshold contact)
%     front caster prismatic carriage on the spring and preload, then a ball on a
%                  spherical joint at the trailing contact point
%     rear caster  rigid, ball on a spherical joint at the trailing contact point
%   The front carriage rests RV.droop above its lower stop and can compress up
%   to RV.travel; caster rolling resistance is in the motor model (Crr on the
%   whole weight), so the swivel joints have no damping.
%   The casters are modelled as ball casters placed where the swivel wheel
%   touches the floor (trail set per run in RV.xF / RV.xR); swivel dynamics are
%   left out.
%
%   Drive, per side (Simulink blocks, averaged PWM, no compiler needed)
%     PI wheel-speed loop -> duty d, limited to +/- RV.pwm_cap
%     i = (d*Vbatt - Ke*w)/R,  T = Kt*i - (T_fric + T_roll)*tanh(w/0.3)
%
%   build_rv_dynamics(P, "simple") draws the bodies as boxes and balls. The
%   default, "cad", draws the SolidWorks parts instead (make_rv_visuals.m); the
%   meshes carry no mass, so both looks give the same results.
%
%   build_rv_dynamics(P, look, "arena") writes rv_arena.slx instead: the same
%   robot in the two-room course of rv_arena_layout.m, steered along its route
%   by rv_pursuit.m, with the chassis footprint checked for collisions with the
%   furniture and walls and its underside for scraping on the ramp, cable cover
%   and threshold.

if nargin < 1, P = rv_params(); end
if nargin < 2, look = "cad"; end
if nargin < 3, scene = "test"; end
if look == "cad"
    try
        V = make_rv_visuals(P);
    catch err
        warning("build_rv_dynamics:visuals", "%s Drawing simple shapes instead.", err.message);
        look = "simple";
    end
end
here = fileparts(mfilename("fullpath"));
mdl = "rv_dynamics";
if scene == "arena", mdl = "rv_arena"; A = rv_arena_layout(P); end
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
load_system("sm_lib"); load_system("nesl_utility"); load_system("simulink");

L.world  = lib("sm_lib/Frames and|Transforms/World Frame");
L.rt     = lib("sm_lib/Frames and|Transforms/Rigid|Transform");
L.mech   = lib("sm_lib/Utilities/Mechanism|Configuration");
L.dof6   = lib("sm_lib/Joints/6-DOF Joint");
L.rev    = lib("sm_lib/Joints/Revolute Joint");
L.pri    = lib("sm_lib/Joints/Prismatic|Joint");
L.sph    = lib("sm_lib/Joints/Spherical|Joint");
L.scf    = lib("sm_lib/Forces and|Torques/Spatial|Contact Force");
L.brick  = lib("sm_lib/Body Elements/Brick Solid");
L.ball   = lib("sm_lib/Body Elements/Spherical Solid");
L.file   = lib("sm_lib/Body Elements/File Solid");
L.cyl    = lib("sm_lib/Body Elements/Cylindrical Solid");
L.tsens  = lib("sm_lib/Frames and|Transforms/Transform|Sensor");
L.solver = "nesl_utility/Solver Configuration";
L.s2ps   = "nesl_utility/Simulink-PS Converter";
L.ps2s   = "nesl_utility/PS-Simulink Converter";

%% Environment
add("world", L.world);
add("mech", L.mech, GravityVector="RV.gvec");
add("solver", L.solver, UseLocalSolver="off");
conn("world","R",1, "mech","L",1);
conn("world","R",1, "solver","R",1);

if scene == "test"
    add("floor_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset="[15 0 -0.01]");
    add("floor", L.brick, ExportEntireGeometry="on", BrickDimensions="[40 6 0.02]", GraphicDiffuseColor="[0.75 0.75 0.72]");
    conn("world","R",1, "floor_T","L",1); conn("floor_T","R",1, "floor","R",1);

    add("thr_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset="[RV.x_thr 0 RV.h_thr/2-0.001]");
    add("threshold", L.brick, ExportEntireGeometry="on", BrickDimensions="[RV.l_thr 6 RV.h_thr+0.002]", GraphicDiffuseColor="[0.55 0.35 0.2]");
    conn("world","R",1, "thr_T","L",1); conn("thr_T","R",1, "threshold","R",1);
    ground = ["floor" "threshold"]; obst = strings(0);
else
    ground = strings(0); obst = strings(0);
    for o = A.objects
        add(o.name + "_T", L.rt, RotationMethod="StandardAxis", RotationStandardAxis=o.axis, RotationAngle=num2str(o.angle), ...
            TranslationMethod="Cartesian", TranslationCartesianOffset=mat2str(o.pos, 6));
        if o.shape == "brick"
            add(o.name, L.brick, ExportEntireGeometry="on", BrickDimensions=mat2str(o.dims, 6), GraphicDiffuseColor=mat2str(o.color));
        else
            add(o.name, L.cyl, ExportEntireGeometry="on", CylinderRadius=num2str(o.dims(1)), CylinderLength=num2str(o.dims(2)), ...
                GraphicDiffuseColor=mat2str(o.color));
        end
        conn("world","R",1, o.name + "_T","L",1); conn(o.name + "_T","R",1, o.name,"R",1);
        if o.role == "ground", ground(end+1) = o.name; else, obst(end+1) = o.name; end %#ok<AGROW>
    end
end

%% Chassis
add("body6dof", L.dof6, PzPositionTargetSpecify="on", PzPositionTargetValue="RV.z0", PzPositionTargetPriority="High", ...
    PxPositionTargetSpecify="on", PxPositionTargetValue="RV.x0", PxPositionTargetPriority="High");
if scene == "test"
    conn("world","R",1, "body6dof","L",1);
else
    add("start_T", L.rt, RotationMethod="StandardAxis", RotationStandardAxis="+Z", RotationAngle="RV.start(3)*180/pi", ...
        TranslationMethod="Cartesian", TranslationCartesianOffset="[RV.start(1) RV.start(2) 0]");
    conn("world","R",1, "start_T","L",1); conn("start_T","R",1, "body6dof","L",1);
end
bc = mean([0 0; P.G.FLOOR_Z P.G.DECK_Z] / 1000, 1);   % visual box centre height
add("chassis_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset=sprintf("[0 0 %.5f]", bc(2)));
add("chassis", L.brick, BrickDimensions=sprintf("[%.4f %.4f %.4f]", P.body(1), P.body(2), (P.G.DECK_Z - P.G.FLOOR_Z)/1000), ...
    InertiaType="Custom", Mass="RV.m_chassis", CenterOfMass=sprintf("RV.com_chassis - [0 0 %.5f]", bc(2)), ...
    MomentsOfInertia="RV.J_moments", ProductsOfInertia="RV.J_products", GraphicDiffuseColor="[0.3 0.45 0.7]", GraphicOpacity="0.6");
conn("body6dof","R",1, "chassis_T","L",1); conn("chassis_T","R",1, "chassis","R",1);

%% Drive wheels and their motors
for side = ["L" "R"]
    sgn = 1; if side == "R", sgn = -1; end
    T = "wheel" + side + "_T";
    add(T, L.rt, RotationMethod="StandardAxis", RotationStandardAxis="+X", RotationAngle="-90", ...
        TranslationMethod="Cartesian", TranslationCartesianOffset=sprintf("[0 %.5f %.5f]", sgn * P.track/2, P.r_wheel));
    J = "wheel" + side + "_joint";
    add(J, L.rev, TorqueActuationMode="InputTorque", SenseVelocity="on", SenseConstraintForce="on", ...
        CompositeWrenchFrame="BaseFrame", DampingCoefficient="0");
    W = "wheel" + side;
    add(W, L.ball, ExportEntireGeometry="on", SphereRadius=sprintf("%.5f", P.r_wheel), InertiaType="Custom", Mass="RV.m_wheel", ...
        CenterOfMass="[0 0 0]", MomentsOfInertia="RV.J_wheel", ProductsOfInertia="[0 0 0]", ...
        GraphicDiffuseColor="[0.15 0.15 0.15]", GraphicOpacity="0.5");
    conn("body6dof","R",1, T,"L",1); conn(T,"R",1, J,"L",1); conn(J,"R",1, W,"R",1);
    for g = ground, contact("c_" + g + "_" + side, g, W); end
    motor(side, J);
end

%% Front caster: spring carriage + ball
add("casterF_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset="[RV.xF 0 RV.zF]");
add("casterF_spring", L.pri, SpringStiffness="RV.k_spring", EquilibriumPosition="-RV.F0_spring/RV.k_spring", ...
    DampingCoefficient="RV.c_spring", LowerLimitSpecify="on", LowerLimitBound="-RV.droop", LowerLimitStiffness="1e6", ...
    LowerLimitDamping="1e3", LowerLimitTransitionRegionWidth="1e-4", UpperLimitSpecify="on", ...
    UpperLimitBound="RV.travel", UpperLimitStiffness="1e6", UpperLimitDamping="1e3", UpperLimitTransitionRegionWidth="1e-4");
add("casterF_carriage", L.brick, BrickDimensions="[0.03 0.03 0.01]", InertiaType="Custom", Mass="RV.m_carriage", ...
    CenterOfMass="[0 0 0]", MomentsOfInertia="[1e-5 1e-5 1e-5]", ProductsOfInertia="[0 0 0]", GraphicDiffuseColor="[0.9 0.6 0.1]");
add("casterF_swivel", L.sph, DampingCoefficient="0");
add("casterF_ball", L.ball, ExportEntireGeometry="on", SphereRadius="RV.r_caster", InertiaType="Custom", Mass="RV.m_cwheel", ...
    CenterOfMass="[0 0 0]", MomentsOfInertia="[1 1 1]*0.4*RV.m_cwheel*RV.r_caster^2", ...
    ProductsOfInertia="[0 0 0]", GraphicDiffuseColor="[0.9 0.6 0.1]", GraphicOpacity="0.6");
conn("body6dof","R",1, "casterF_T","L",1); conn("casterF_T","R",1, "casterF_spring","L",1);
conn("casterF_spring","R",1, "casterF_carriage","R",1); conn("casterF_spring","R",1, "casterF_swivel","L",1);
conn("casterF_swivel","R",1, "casterF_ball","R",1);
for g = ground, contact("c_" + g + "_F", g, "casterF_ball"); end

%% Rear caster: rigid + ball
add("casterB_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset="[RV.xR 0 RV.r_caster]");
add("casterB_swivel", L.sph, DampingCoefficient="0");
add("casterB_ball", L.ball, ExportEntireGeometry="on", SphereRadius="RV.r_caster", InertiaType="Custom", Mass="RV.m_rear", ...
    CenterOfMass="[0 0 0]", MomentsOfInertia="[1 1 1]*0.4*RV.m_rear*RV.r_caster^2", ...
    ProductsOfInertia="[0 0 0]", GraphicDiffuseColor="[0.9 0.6 0.1]", GraphicOpacity="0.6");
conn("body6dof","R",1, "casterB_T","L",1); conn("casterB_T","R",1, "casterB_swivel","L",1);
conn("casterB_swivel","R",1, "casterB_ball","R",1);
for g = ground, contact("c_" + g + "_B", g, "casterB_ball"); end

%% CAD look: hide the primitives and hang the part meshes on the moving bodies
if look == "cad"
    for b = ["chassis" "wheelL" "wheelR" "casterF_carriage" "casterF_ball" "casterB_ball"]
        set_param(mdl + "/" + b, GraphicType="None");
    end
    add("drawer_vis_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset="[-RV.drawer_out 0 0]");
    conn("body6dof","R",1, "drawer_vis_T","L",1);
    add("carriage_vis_T", L.rt, TranslationMethod="Cartesian", TranslationCartesianOffset="-[RV.xF 0 RV.zF]");
    conn("casterF_spring","R",1, "carriage_vis_T","L",1);
    mount = dictionary(["chassis" "drawer" "carriage" "wheelL" "wheelR"], ...
        ["body6dof" "drawer_vis_T" "carriage_vis_T" "wheelL_joint" "wheelR_joint"]);   % follower side R of each
    for v = V
        n = "vis_" + v.name;
        add(n, L.file, ExtGeomFileName=v.file, UnitType="Custom", ExtGeomFileUnits="m", ...
            InertiaType="PointMass", Mass="0", GraphicDiffuseColor=mat2str(v.color), ...
            GraphicOpacity=num2str(v.opacity));
        conn(mount(v.body), "R", 1, n, "R", 1);
    end
end

%% Speed commands
if scene == "test"
    add("ref", "simulink/Sources/From Workspace", VariableName="RV_ref", OutputAfterFinalValue="Holding final value");
else
    % chassis pose in the world -> pure pursuit at 50 Hz -> acceleration limits -> [v yaw_rate]
    add("pose_xyz", L.tsens, SenseX="on", SenseY="on", SenseZ="on");
    add("pose_q", L.tsens, SenseQ="on");
    for s = ["pose_xyz" "pose_q"], conn("world","R",1, s,"L",1); conn("body6dof","R",1, s,"R",1); end
    ax = ["px" "py" "pz"];
    for j = 1:3
        add(ax(j), L.ps2s, Unit="m"); conn("pose_xyz","R",j+1, ax(j),"L",1);
    end
    add("pq", L.ps2s, Unit="1"); conn("pose_q","R",2, "pq","L",1);
    add("clock", "simulink/Sources/Clock");
    add("pin", "simulink/Signal Routing/Mux", Inputs="4");
    add_line(mdl, "clock/1", "pin/1"); add_line(mdl, "px/1", "pin/2"); add_line(mdl, "py/1", "pin/3"); add_line(mdl, "pq/1", "pin/4");
    add("pzoh", "simulink/Discrete/Zero-Order Hold", SampleTime="0.02");
    add_line(mdl, "pin/1", "pzoh/1");
    add("pursuit", "simulink/User-Defined Functions/Interpreted MATLAB Function", MATLABFcn="rv_pursuit(u)", OutputDimensions="2");
    add_line(mdl, "pzoh/1", "pursuit/1");
    add("pdmx", "simulink/Signal Routing/Demux", Outputs="2");
    add_line(mdl, "pursuit/1", "pdmx/1");
    add("vlim", "simulink/Discontinuities/Rate Limiter", RisingSlewLimit="0.5", FallingSlewLimit="-0.8");
    add("wlim", "simulink/Discontinuities/Rate Limiter", RisingSlewLimit="4", FallingSlewLimit="-4");
    add_line(mdl, "pdmx/1", "vlim/1"); add_line(mdl, "pdmx/2", "wlim/1");
    add("ref", "simulink/Signal Routing/Mux", Inputs="2");
    add_line(mdl, "vlim/1", "ref/1"); add_line(mdl, "wlim/1", "ref/2");

    % collision and scrape checks: a massless box over the robot's plan extents (tyres
    % and charge pads included), from the lowest point of the chassis up to the deck; it touches obstacles (collision) and ground objects other
    % than the floor (underside scraping)
    B = A.box;                                     % [xmin xmax ymax zmin ztop] from the CAD
    add("bumpbox_T", L.rt, TranslationMethod="Cartesian", ...
        TranslationCartesianOffset=sprintf("[%.5f 0 %.5f]", mean(B(1:2)), mean(B(4:5))));
    add("bumpbox", L.brick, ExportEntireGeometry="on", BrickDimensions=sprintf("[%.4f %.4f %.4f]", ...
        diff(B(1:2)), 2*B(3), B(5) - B(4)), InertiaType="PointMass", Mass="0", GraphicType="None");
    conn("body6dof","R",1, "bumpbox_T","L",1); conn("bumpbox_T","R",1, "bumpbox","R",1);
    for o = obst, contact("k_" + o, o, "bumpbox"); end
    scrape = setdiff(ground, "floor", "stable");
    for g = scrape, contact("k_" + g, g, "bumpbox"); end
    grp = {"fcoll", obst; "fscrape", scrape};
    for r = 1:2
        nm = grp{r,1}; lst = grp{r,2};
        add(nm, "simulink/Math Operations/Sum", Inputs=repmat('+', 1, numel(lst)));
        for j = 1:numel(lst), add_line(mdl, "fn_k_" + lst(j) + "/1", nm + "/" + j); end
    end
end
add("ref_demux", "simulink/Signal Routing/Demux", Outputs="2");
add_line(mdl, "ref/1", "ref_demux/1");
for side = ["L" "R"]
    sgn = -1; if side == "R", sgn = 1; end      % left wheel slower in a left (positive yaw) turn
    add("wref_" + side, "simulink/User-Defined Functions/Fcn", ...
        Expr=sprintf("(u(1) %+g*u(2))/%.6g", sgn * P.track/2, P.r_wheel));
    add("refmux_" + side, "simulink/Signal Routing/Mux", Inputs="2");
    add_line(mdl, "ref_demux/1", "refmux_" + side + "/1");
    add_line(mdl, "ref_demux/2", "refmux_" + side + "/2");
    add_line(mdl, "refmux_" + side + "/1", "wref_" + side + "/1");
    add_line(mdl, "wref_" + side + "/1", "err_" + side + "/1");
end

%% Logging
nlog = 22; if scene == "arena", nlog = 30; end
add("log_mux", "simulink/Signal Routing/Mux", Inputs=num2str(nlog));
add("log", "simulink/Sinks/To Workspace", VariableName="rvlog", SaveFormat="Timeseries");
add_line(mdl, "log_mux/1", "log/1");
% channels: 1 wL 2 wR 3 wrefL 4 wrefR 5 dL 6 dR 7 iL 8 iR 9 TL 10 TR
%           11 FnL 12 FnR 13 FnF 14 FnB (floor + threshold) 15-17 fcL 18-20 fcR 21 q_spring 22 Vbatt
k = 0;
for side = ["L" "R"], k = k + 1; add_line(mdl, "w_" + side + "/1", "log_mux/" + k); end
for side = ["L" "R"], k = k + 1; add_line(mdl, "wref_" + side + "/1", "log_mux/" + k); end
for side = ["L" "R"], k = k + 1; add_line(mdl, "pid_" + side + "/1", "log_mux/" + k); end
for side = ["L" "R"], k = k + 1; add_line(mdl, "cur_" + side + "/1", "log_mux/" + k); end
for side = ["L" "R"], k = k + 1; add_line(mdl, "torq_" + side + "/1", "log_mux/" + k); end
for c = ["L" "R" "F" "B"]
    k = k + 1;
    add("fn_sum_" + c, "simulink/Math Operations/Sum", Inputs=repmat('+', 1, numel(ground)));
    for j = 1:numel(ground), add_line(mdl, "fn_c_" + ground(j) + "_" + c + "/1", "fn_sum_" + c + "/" + j); end
    add_line(mdl, "fn_sum_" + c + "/1", "log_mux/" + k);
end
for side = ["L" "R"]
    add("fc_dmx_" + side, "simulink/Signal Routing/Demux", Outputs="3");
    add_line(mdl, "fc_" + side + "/1", "fc_dmx_" + side + "/1");
    for j = 1:3, k = k + 1; add_line(mdl, "fc_dmx_" + side + "/" + j, "log_mux/" + k); end
end
% spring compression sensor
set_param(mdl + "/casterF_spring", SensePosition="on");
add("q_spring", L.ps2s, Unit="m");
conn("casterF_spring","R",2, "q_spring","L",1);
k = k + 1; add_line(mdl, "q_spring/1", "log_mux/" + k);
k = k + 1; add_line(mdl, "vbatt/1", "log_mux/" + k);
if scene == "arena"
    % channels 23-25 x y z (world), 26-29 Q, 30 collision force, 31 scrape force, 32-33 v and yaw rate commands
    for s = ["px" "py" "pz" "pq" "fcoll" "fscrape" "vlim" "wlim"], k = k + 1; add_line(mdl, s + "/1", "log_mux/" + k); end
end

%% Solver and logging settings
stop = "10"; if scene == "arena", stop = num2str(A.tstop); end
set_param(mdl, StopTime=stop, Solver="ode23t", MaxStep="2e-3", RelTol="1e-3", AbsTol="1e-5", ...
    ReturnWorkspaceOutputs="on", SignalLogging="off");
set_param(mdl, ZeroCrossControl="DisableAll");   % PI saturation chatters at full-power steps otherwise
if scene == "test"
    set_param(mdl, SimscapeLogType="all");
else
    set_param(mdl, SimscapeLogType="none");      % the arena logs its pose through sensors; full logging slows it down
end
set_param(mdl, SimscapeLogDecimation=5);
set_param(mdl, SimMechanicsOpenEditorOnUpdate="off");
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(here, mdl + ".slx"));
fprintf("saved %s\n", fullfile(here, mdl + ".slx"));

%% ---------------------------------------------------------------- helpers
    function h = add(name, src, varargin)
        h = add_block(src, mdl + "/" + name);
        if ~isempty(varargin), set_param(h, varargin{:}); end
    end

    function conn(a, sa, ia, b, sb, ib)
        pa = get_param(mdl + "/" + a, "PortHandles");
        pb = get_param(mdl + "/" + b, "PortHandles");
        add_line(mdl, pick(pa, sa, ia), pick(pb, sb, ib), "autorouting", "off");
    end

    function p = pick(ph, s, i)
        % single-port utility blocks put their port on either side
        if s == "L" && isempty(ph.LConn), s = "R"; end
        if s == "R" && isempty(ph.RConn), s = "L"; end
        switch s
            case "L", p = ph.LConn(i);
            case "R", p = ph.RConn(i);
        end
    end

    function contact(name, base, foll)
        add(name, L.scf, NormalStiffness="RV.k_contact", NormalDamping="RV.c_contact", ...
            NormalTransitionRegionWidth="1e-4", FrictionType="SmoothStickSlip", ...
            CoefficientOfStaticFriction="RV.mu_s", CoefficientOfDynamicFriction="RV.mu_d", ...
            FrictionalCriticalVelocity="1e-2", SenseNormalForceMagnitude="on");
        % contact uses the solids' geometry port G (left side once Export entire geometry is on)
        conn(base,"L",1, name,"L",1);
        conn(foll,"L",1, name,"R",1);
        add("fn_" + name, L.ps2s, Unit="N");
        conn(name,"R",2, "fn_" + name,"L",1);
    end

    function motor(side, J)
        M = P.motor;
        % wheel speed out of the joint
        add("w_" + side, L.ps2s, Unit="rad/s");
        conn(J,"R",2, "w_" + side,"L",1);
        add("fc_" + side, L.ps2s, Unit="N");
        conn(J,"R",3, "fc_" + side,"L",1);
        % speed loop
        add("err_" + side, "simulink/Math Operations/Sum", Inputs="+-");
        add_line(mdl, "w_" + side + "/1", "err_" + side + "/2");
        add("pid_" + side, "simulink/Continuous/PID Controller", Controller="PI", P="RV.Kp", I="RV.Ki", ...
            LimitOutput="on", UpperSaturationLimit="RV.pwm_cap", LowerSaturationLimit="-RV.pwm_cap", ...
            AntiWindupMode="clamping");
        add_line(mdl, "err_" + side + "/1", "pid_" + side + "/1");
        % averaged H-bridge + DC motor referred to the gearbox output
        if side == "L"
            add("vbatt", "simulink/Sources/Constant", Value="RV.Vbatt");
        end
        add("mmux_" + side, "simulink/Signal Routing/Mux", Inputs="3");
        add_line(mdl, "pid_" + side + "/1", "mmux_" + side + "/1");
        add_line(mdl, "w_" + side + "/1", "mmux_" + side + "/2");
        add_line(mdl, "vbatt/1", "mmux_" + side + "/3");
        add("cur_" + side, "simulink/User-Defined Functions/Fcn", ...
            Expr=sprintf("(u(1)*u(3) - %.6g*u(2))/%.6g", M.Ke, M.R));
        add_line(mdl, "mmux_" + side + "/1", "cur_" + side + "/1");
        add("tmux_" + side, "simulink/Signal Routing/Mux", Inputs="2");
        add_line(mdl, "cur_" + side + "/1", "tmux_" + side + "/1");
        add_line(mdl, "w_" + side + "/1", "tmux_" + side + "/2");
        Troll = P.Crr * P.m_total * P.g / 2 * P.r_wheel;
        add("torq_" + side, "simulink/User-Defined Functions/Fcn", ...
            Expr=sprintf("%.6g*u(1) - %.6g*tanh(u(2)/0.3)", M.Kt, M.T_fric + Troll));
        add_line(mdl, "tmux_" + side + "/1", "torq_" + side + "/1");
        add("tq_" + side, L.s2ps, Unit="N*m");
        add_line(mdl, "torq_" + side + "/1", "tq_" + side + "/1");
        conn("tq_" + side,"R",1, J,"L",2);
    end
end

function p = lib(s)
p = strrep(s, "|", newline);
end
