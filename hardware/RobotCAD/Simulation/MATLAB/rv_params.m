function P = rv_params()
%RV_PARAMS  All RoamView parameters for the MATLAB simulations, in SI units.
%   P = rv_params() reads Admin/RV_globals.txt, Simulation/MassProps/mass_table.csv
%   and Simulation/MassProps/mass_inertia.csv, and adds the drivetrain constants
%   used by hand_calcs.py. Every script calls this, so a globals change flows
%   through on the next run.
%
%   Frame: ROS REP-103, origin on the floor under the drive axle,
%   x forward, y left, z up.
%
%   The RobotCAD folder is found in this order: the environment variable
%   RV_ROBOTCAD, the folder two levels up (the normal location), then the main
%   RoamView checkout when this file sits in a .claude/worktrees copy.

P.root = find_root();
P.dirs.sim = fullfile(P.root, "Simulation");
P.dirs.motion = fullfile(P.dirs.sim, "Motion");
P.dirs.fea = fullfile(P.dirs.sim, "FEA");
P.dirs.step = fullfile(P.root, "Exports", "STEP");
P.dirs.stl = fullfile(P.root, "Exports", "STL");
P.dirs.out = fullfile(fileparts(mfilename("fullpath")), "results");   % reports and plots
if ~isfolder(P.dirs.out), mkdir(P.dirs.out); end

%% Globals (mm, as in SolidWorks)
G = read_globals(fullfile(P.root, "Admin", "RV_globals.txt"));
P.G = G;

%% Masses
T = readtable(fullfile(P.dirs.sim, "MassProps", "mass_table.csv"), TextType="string");
mg = @(pn) T.Mass_g(T.PartNo == pn) / 1000;
P.mass_table = T;

I = readtable(fullfile(P.dirs.sim, "MassProps", "mass_inertia.csv"), TextType="string");
q = @(name) I.value(I.quantity == name);
P.m_total = q("mass");
P.com = [q("com_x") q("com_y") q("com_z")] / 1000;
P.m_drawer = q("drawer_mass");
P.J_total = [q("Ixx") q("Ixy") q("Ixz"); q("Ixy") q("Iyy") q("Iyz"); q("Ixz") q("Iyz") q("Izz")];

%% Geometry (m)
mm = 1e-3;
P.r_wheel = G.WHEEL_D / 2 * mm;
P.w_wheel = G.WHEEL_W * mm;
P.track = G.TRACK * mm;
P.caster_x = G.CASTER_X * mm;          % swivel axis, front at +x, rear at -x
P.caster_trail = G.CASTER_TRAIL * mm;
P.r_caster = G.CASTER_D / 2 * mm;
P.w_caster = G.CASTER_W * mm;
P.caster_h = G.CASTER_H * mm;          % swivel top to floor
P.body = [G.BODY_L G.BODY_W] * mm;
P.spring_travel = 10 * mm;             % front caster carriage travel (sweeps)

%% Moving bodies split from the chassis
P.m_wheel = mg("RV-503") + mg("RV-502");             % wheel + hub, per side
P.m_caster = mg("RV-504");                           % whole goBILDA caster
P.m_caster_wheel = 0.35 * P.m_caster;                % tyre + axle share (estimate)
P.m_caster_fork = P.m_caster - P.m_caster_wheel;
P.m_carriage = mg("RV-309");                         % sprung carriage of the front caster

% chassis = total minus the separately modelled bodies; COM and inertia corrected
% with the parallel axis theorem so the assembled robot matches SolidWorks.
parts = [ ...
    P.m_wheel,  0,  P.track/2, P.r_wheel;
    P.m_wheel,  0, -P.track/2, P.r_wheel;
    P.m_caster + P.m_carriage,  P.caster_x, 0, P.caster_h/2;
    P.m_caster, -P.caster_x, 0, P.caster_h/2];
mc = P.m_total - sum(parts(:,1));
cc = (P.m_total * P.com - sum(parts(:,1) .* parts(:,2:4), 1)) / mc;
Jc = P.J_total + pa(P.m_total, P.com - cc);       % total, about the chassis COM
for k = 1:size(parts,1)
    Jc = Jc - pa(parts(k,1), parts(k,2:4) - cc);  % take the separate bodies out
end
P.m_chassis = mc;
P.com_chassis = cc;
P.J_chassis = Jc;
P.parts_split = parts;

%% Drivetrain (Pololu 4754 37D 70:1 at 12 V, as in hand_calcs.py)
M.V_rated = 12;
M.T_stall = 2.65;          % N.m at the gearbox output
M.I_stall = 5.5;           % A
M.w_free = 150 * 2*pi/60;  % rad/s at the output
M.I_free = 0.2;            % A
M.T_cont = 0.98;           % continuous limit, N.m
M.T_inst = 2.45;           % instantaneous limit, N.m
M.gear = 70;
M.eta = 0.9;
% lumped DC motor constants referred to the gearbox output
M.R = M.V_rated / M.I_stall;                       % ohm
M.Kt = M.T_stall / (M.I_stall - M.I_free);         % N.m/A at the output
M.Ke = (M.V_rated - M.I_free * M.R) / M.w_free;    % V.s/rad at the output
M.T_fric = M.Kt * M.I_free;                        % no-load loss torque at the output
M.L = 1.5e-3;                                      % H (sets the current time constant only)
M.J_out = 1.0e-6 * M.gear^2;                       % rotor inertia seen at the output (estimate)
P.motor = M;

P.driver.I_max = 12;           % DFR0601, A per channel
P.driver.pwm_cap = 0.71;       % recommendation 3 in the validation report

P.batt.V_full = 16.8;
P.batt.V_nom = 14.8;
P.batt.V_empty = 12.8;
P.batt.R_int = 0.040;          % ohm, 4S Li-ion pack (placeholder)
P.batt.capacity_Ah = 5.0;      % PLACEHOLDER: pack not chosen yet (900 g envelope)
P.elec_load_W = 18;            % PLACEHOLDER: Pi 5 + OAK-D Pro W + LiDAR + ESP32 + display, average

%% Ground and contact
P.g = 9.81;
P.Crr = 0.02;
P.mu = 0.7;

%% Front caster spring (recommendation 1: 2 x 1.5 N/mm, 6 N preload each)
P.spring.k = 2 * 1.5e3;        % N/m, both springs
P.spring.F0 = 2 * 6;           % N, both springs
P.spring.c = 60;               % N/(m/s), carriage friction stand-in
P.spring.preset = 0.5e-3;      % m the front tyre sits below the floor line with the carriage on
                               % its stop, so at rest it carries about the preload (hand_calcs.py: ~15 N)

%% Motor speed loop (PI on wheel speed, duty out); see build_rv_dynamics.m
P.ctrl.Kp = 0.12;              % duty per rad/s
P.ctrl.Ki = 0.8;               % duty per rad
end

%% ------------------------------------------------------------------------
function root = find_root()
here = fileparts(mfilename("fullpath"));
cands = string(getenv("RV_ROBOTCAD"));
cands(end+1) = fullfile(here, "..", "..");
% worktree copy: RoamView/.claude/worktrees/<name>/hardware/RobotCAD/Simulation/MATLAB
cands(end+1) = fullfile(here, "..", "..", "..", "..", "..", "..", "..", "hardware", "RobotCAD");
for c = cands
    if strlength(c) > 0 && isfile(fullfile(c, "Admin", "RV_globals.txt"))
        root = string(char(java.io.File(c).getCanonicalPath()));
        return
    end
end
error("rv_params:root", "RobotCAD folder not found. Set RV_ROBOTCAD to the RobotCAD path.");
end

function G = read_globals(file)
% Same rule as fea.py read_globals(): a global may reference globals above it.
lines = readlines(file);
names = strings(0); exprs = strings(0);
for ln = lines'
    if contains(ln, "=")
        parts = split(ln, "=");
        names(end+1) = strip(strip(parts(1)), '"'); %#ok<AGROW>
        exprs(end+1) = strip(join(parts(2:end), "=")); %#ok<AGROW>
    end
end
G = struct();
pending = true(size(names));
while any(pending)
    progressed = false;
    for i = find(pending)
        tok = regexp(exprs(i), '"([^"]+)"', "tokens");
        refs = strings(0);
        for t = 1:numel(tok), refs(end+1) = string(tok{t}{1}); end %#ok<AGROW>
        if all(isfield(G, refs))
            e = exprs(i);
            [~, order] = sort(strlength(refs), "descend");
            for r = refs(order)
                e = replace(e, """" + r + """", sprintf("(%.12g)", G.(r)));
            end
            G.(names(i)) = eval(e);
            pending(i) = false;
            progressed = true;
        end
    end
    if ~progressed
        error("rv_params:globals", "Unresolvable globals: %s", strjoin(names(pending), ", "));
    end
end
end

function J = pa(m, d)
% parallel axis term m*(|d|^2 I - d d')
d = d(:);
J = m * (dot(d, d) * eye(3) - (d * d'));
end
