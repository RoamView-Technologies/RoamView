function R = rv_sim(RV, ref, tstop, show)
%RV_SIM  Run rv_dynamics.slx once and return its signals on a common time base.
%   R = rv_sim(RV, ref, tstop)
%   R = rv_sim(RV, ref, tstop, pace) also animates the run in Mechanics Explorer,
%     paced at pace x real time (true = 1, 0.25 = slow motion, 0 = no animation).
%     Without the argument, the pace set by run_all(animate=true) applies.
%     RV     settings from rv_tunables
%     ref    [t v yaw_rate] speed command table (m/s, rad/s)
%     tstop  stop time, s
%   R has fields t, x, z, vx, pitch, roll, yaw (chassis), w/wref/d/i/T/Tload
%   (2 columns, left then right; T is the net joint torque, Tload adds back the
%   rolling resistance the motor model subtracts), Fn (4 columns: left, right, front caster, rear caster),
%   fc (wheel joint constraint force, 3 per side, joint base frame: z = axle),
%   q_spring (front caster carriage compression, m), Vbatt; Fn_s and ax are
%   the contact loads and chassis acceleration smoothed over 20 / 50 ms.
%   Signals are resampled to a uniform 1 ms grid.

mdl = "rv_dynamics"; scene = "test";
if isfield(RV, "model"), mdl = RV.model; scene = RV.scene; end
here = fileparts(mfilename("fullpath"));
if ~bdIsLoaded(mdl)
    if ~isfile(fullfile(here, mdl + ".slx")), build_rv_dynamics(rv_params(), "cad", scene); end
    load_system(fullfile(here, mdl + ".slx"));
end
in = Simulink.SimulationInput(mdl);
in = in.setVariable("RV", RV);
in = in.setVariable("RV_ref", ref);
in = in.setModelParameter("StopTime", num2str(tstop));
if nargin < 4, show = getappdata(0, "rv_animate"); end
if isempty(show) || ~usejava("desktop"), show = 0; end   % nothing to draw in under matlab -batch
if show
    in = in.setModelParameter("SimMechanicsOpenEditorOnUpdate", "on", ...
        "EnablePacing", "on", "PacingRate", num2str(double(show)));
    st = dbstack(1); caller = "command line"; if ~isempty(st), caller = string(st(1).name); end
    thr = "none"; if RV.x_thr < 100, thr = sprintf("at x = %.2f m", RV.x_thr); end
    fprintf("  [animating] %s: %.1f s, ramp %.0f deg, threshold %s, spring %.2f N/mm with %.0f N each, drawer out %.0f mm\n", ...
        caller, tstop, atand(-RV.gvec(1) / -RV.gvec(3)), thr, RV.k_spring / 2e3, RV.F0_spring / 2, 1000 * RV.drawer_out);
end
out = sim(in);
if ~isempty(out.ErrorMessage), error("rv_sim:failed", "%s", out.ErrorMessage); end

lg = out.rvlog;
t = lg.Time;
Y = squeeze(lg.Data);
if size(Y, 1) ~= numel(t), Y = Y.'; end
[t, iu] = unique(t, "last");               % drop repeated event times
Y = Y(iu, :);
% uniform 1 ms grid so derivatives and moving averages behave
tu = (t(1):1e-3:t(end))';
Y = interp1(t, Y, tu, "linear");
t = tu;
R.t = t;
R.w = Y(:, 1:2);  R.wref = Y(:, 3:4);  R.d = Y(:, 5:6);
R.i = Y(:, 7:8);  R.T = Y(:, 9:10);    R.Fn = Y(:, 11:14);
R.fc = Y(:, 15:20); R.q_spring = Y(:, 21); R.Vbatt = Y(:, 22);
if size(Y, 2) >= 33                          % arena: world pose, checks and commands
    R.xw = Y(:, 23); R.yw = Y(:, 24); R.zw = Y(:, 25);
    Qw = Y(:, 26:29) ./ vecnorm(Y(:, 26:29), 2, 2);
    R.Qw = Qw;
    [R.yaw_w, R.pitch_w, R.roll_w] = quat2eul_zyx(Qw);
    R.Fcoll = Y(:, 30); R.Fscrape = Y(:, 31); R.vcmd = Y(:, 32); R.wcmd = Y(:, 33);
end

% chassis pose from the Simscape log of the 6-DOF joint
if isfield(R, "xw")                           % arena: world pose from the sensors
    R.x = R.xw; R.y = R.yw; R.z = R.zw;
    R.yaw = R.yaw_w; R.pitch = R.pitch_w; R.roll = R.roll_w;
    vxy = [gradient(movmean(R.xw, 20), t), gradient(movmean(R.yw, 20), t)];
    R.vx = sum(vxy .* [cos(R.yaw) sin(R.yaw)], 2);   % forward speed
else
    sl = out.(get_param(mdl, "SimscapeLogName")).body6dof;
    R.x  = resamp(sl.Px.p, t);  R.vx = resamp(sl.Px.v, t);
    R.y  = resamp(sl.Py.p, t);
    R.z  = resamp(sl.Pz.p, t);
    Q = resamp(sl.S.Q, t);                       % [w x y z]
    [R.yaw, R.pitch, R.roll] = quat2eul_zyx(Q);
end
R.Tload = R.T + RV.T_roll * tanh(R.w / 0.3);   % load torque at the output, incl. rolling resistance
R.Fn_s = movmean(R.Fn, 20);                 % contact loads averaged over 20 ms (drops impact spikes)
R.ax = gradient(movmean(R.vx, 50), t);      % chassis acceleration from speed averaged over 50 ms
R.ref = ref;
R.RV = RV;
end

function v = resamp(node, t)
ts = node.series;
v = interp1(ts.time, ts.values(ts.unit), t, "linear", "extrap");
end

function [yaw, pitch, roll] = quat2eul_zyx(Q)
qw = Q(:,1); qx = Q(:,2); qy = Q(:,3); qz = Q(:,4);
yaw = atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));
pitch = asin(max(-1, min(1, 2*(qw.*qy - qz.*qx))));
roll = atan2(2*(qw.*qx + qy.*qz), 1 - 2*(qx.^2 + qy.^2));
end
