function S = run_dynamics(P)
%RUN_DYNAMICS  Drive RoamView through the validation cases in rv_dynamics.slx.
%   S = run_dynamics() runs every case, compares it with the drivetrain hand
%   calcs (Simulation/Motion/hand_calcs_drivetrain.txt), and writes
%   results/matlab_dynamics.txt plus one PNG per case. S holds the raw runs.

if nargin < 1, P = rv_params(); end
M = P.motor;
out = strings(0);
W = P.m_total * P.g;
t0 = 0.5;                                  % settle before any command

out(end+1) = "RoamView dynamics in Simscape Multibody (rv_dynamics.slx), MATLAB " + version("-release");
out(end+1) = sprintf("Robot %.2f kg, COM x %.1f z %.1f mm; motor %s; PWM cap %.0f %% at %.1f V; mu %.2f, Crr %.2f", ...
    P.m_total, 1000*P.com(1), 1000*P.com(3), "Pololu 37D 70:1", 100*P.driver.pwm_cap, P.batt.V_full, P.mu, P.Crr);
out(end+1) = "Torque is the load torque at each gearbox output; current includes the 0.2 A no-load current.";
out(end+1) = "";

%% 0. static load split
R = rv_sim(rv_tunables(P), prof(t0, []), 1.0);
Fs = mean(R.Fn(R.t > 0.8, :));
S.static = R;
out(end+1) = "0. Standing still (flat)";
out(end+1) = sprintf("   drive wheels %.1f / %.1f N, front caster %.1f N, rear caster %.1f N (sum %.1f N, weight %.1f N)", ...
    Fs, sum(Fs), W);
out(end+1) = sprintf("   front carriage compression %.2f mm (preload %.0f N, %.1f N/mm)", 1000*R.q_spring(end), P.spring.F0, P.spring.k/1000);
out(end+1) = "   hand calc ignored the front preload (42.0 N per wheel, rear 1.2 N); the spring moves ~13 N onto each caster.";
out(end+1) = "";

%% 1. flat: accelerate, cruise, brake
ref = prof(t0, [1.0 0.5 0; 2.0 0.5 0; 0.5 0 0; 1.5 0 0]);
R = rv_sim(rv_tunables(P), ref, 6.0);
S.flat = R;
ia = win(R, t0 + [0.3 0.9]); ic = win(R, t0 + [1.8 3.0]);
tb = t0 + 3.0;
[dstop, tstop] = stopdist(R, tb);
ib = win(R, tb + [0 0.5]);
out(end+1) = "1. Flat floor: 0.5 m/s^2 to 0.5 m/s, cruise, brake at 1 m/s^2";
out(end+1) = row("cruise 0.5 m/s", mean(R.Tload(ic,:), "all"), mean(R.i(ic,:), "all"), 0.07, 0.34, M.T_cont);
out(end+1) = row("accelerate 0.5 m/s^2", mean(R.Tload(ia,:), "all"), mean(R.i(ia,:), "all"), 0.24, 0.68, M.T_cont);
out(end+1) = sprintf("   braking: stop in %.0f mm after the command (hand calc 125 mm), %.2f s; regen torque %.2f N.m/motor", ...
    1000*dstop, tstop, -min(mean(R.Tload(ib,:), 2)));
out(end+1) = sprintf("   braking pitch %.2f deg nose down, front carriage compression max %.1f mm (hand calc ~3 mm), front caster %.1f N max", ...
    rad2deg(max(R.pitch(ib))), 1000*max(R.q_spring(ib)), max(R.Fn(ib,3)));
out(end+1) = sprintf("   rear caster load min %.1f N while braking (static %.1f N): %s", min(R.Fn_s(ib,4)), Fs(4), ...
    pf(min(R.Fn_s(ib,4)) > 1, "stays on the floor", "LIFTS"));
out(end+1) = "   braking load transfer comes off the rigid rear caster first; the front spring barely moves while the rear stays loaded.";
out(end+1) = "";
plotcase(R, "flat", "Flat: accelerate, cruise 0.5 m/s, brake 1 m/s^2", P);

%% 2. 5 degree ramp
ref = prof(t0, [0.6 0.3 0; 2.5 0.3 0]);
R = rv_sim(rv_tunables(P, ramp_deg=5), ref, 4.0);
S.ramp = R;
ia = win(R, t0 + [0.15 0.55]); ic = win(R, t0 + [1.5 3.0]);
out(end+1) = "2. 5 deg ramp, uphill: 0.5 m/s^2 to 0.3 m/s, cruise";
out(end+1) = row("cruise 0.3 m/s", mean(R.Tload(ic,:), "all"), mean(R.i(ic,:), "all"), 0.36, 0.93, M.T_cont);
out(end+1) = row("accelerate 0.5 m/s^2", mean(R.Tload(ia,:), "all"), mean(R.i(ia,:), "all"), 0.54, 1.28, M.T_cont);
out(end+1) = sprintf("   rolled back %.1f mm while holding still before the start", -1000*min(0, min(R.x(R.t < t0)) - R.x(1)));
out(end+1) = "";
plotcase(R, "ramp5", "5 deg ramp: 0.3 m/s uphill", P);

%% 3. 15 mm door threshold, square edge
out(end+1) = "3. 15 mm square-edged threshold (40 mm long) at 0.3 m/s";
ks = P.spring.k; Fs0 = P.spring.F0; kso = 2*0.4e3; Fso = 2*3;    % design spring; soft option 0.4 N/mm, 3 N each
thr = {"forward, as designed (front sprung caster leads)", 1, 2*P.r_caster, P.spring_travel, 0, ks, Fs0, "thr_fwd";
       "reverse, as designed (rear rigid caster leads)", -1, 2*P.r_caster, P.spring_travel, 0, ks, Fs0, "thr_rev";
       "forward, 3-inch casters (CASTER_D 75 option)", 1, 0.075, P.spring_travel, 0, ks, Fs0, "thr_fwd_3in";
       "forward, design spring, carriage 18 mm travel + 12 mm droop", 1, 2*P.r_caster, 0.018, 0.012, ks, Fs0, "thr_fwd_travel";
       "forward, soft spring", 1, 2*P.r_caster, P.spring_travel, 0, kso, Fso, "thr_fwd_soft";
       "reverse, soft spring", -1, 2*P.r_caster, P.spring_travel, 0, kso, Fso, "thr_rev_soft"};
out(end+1) = "   design spring = 2 x 1.5 N/mm, 6 N preload each; soft spring = 2 x 0.4 N/mm, 3 N preload each";
for k = 1:size(thr, 1)
    d = thr{k,2};
    x_edge = d * 0.35;  if d < 0, x_edge = x_edge - 0.04; end
    RV = rv_tunables(P, x_thr=x_edge, direction=d, caster_d=thr{k,3}, travel=thr{k,4}, droop=thr{k,5}, ...
        k_spring=thr{k,6}, F0_spring=thr{k,7});
    ref = prof(t0, [0.6 0.3*d 0; 4.0 0.3*d 0]);
    R = rv_sim(RV, ref, 5.0);
    S.(thr{k,8}) = R;
    goal = abs(x_edge) + 0.04 + P.caster_x + P.caster_trail + 0.06;   % rear-most contact past the far edge
    ok = abs(R.x(end)) > goal;
    imv = R.t > t0 + 0.8;
    vmin = min(d * R.vx(imv));
    out(end+1) = sprintf("   %s: %s (travelled %.0f mm of %.0f needed)", thr{k,1}, pf(ok, "CROSSES", "STUCK"), ...
        1000*abs(R.x(end)), 1000*goal); %#ok<AGROW>
    out(end+1) = sprintf("      min speed %.2f m/s, peak torque %.2f N.m (%.0f %% of instantaneous %.2f), peak current %.1f A", ...
        vmin, max(abs(R.Tload(imv,:)), [], "all"), 100*max(abs(R.Tload(imv,:)), [], "all")/M.T_inst, M.T_inst, max(abs(R.i(imv,:)), [], "all")); %#ok<AGROW>
    out(end+1) = sprintf("      peak caster contact force front %.0f N / rear %.0f N, carriage compression max %.1f mm (stop at %.0f mm), pitch max %.1f deg", ...
        max(R.Fn_s(imv,3)), max(R.Fn_s(imv,4)), 1000*max(R.q_spring(imv)), 1000*thr{k,4}, rad2deg(max(abs(R.pitch(imv))))); %#ok<AGROW>
    out(end+1) = sprintf("      carriage range used %.1f to %.1f mm (droop limit -%.0f mm)", 1000*min(R.q_spring(imv)), ...
        1000*max(R.q_spring(imv)), 1000*thr{k,5}); %#ok<AGROW>
    plotcase(R, thr{k,8}, "Threshold 15 mm: " + thr{k,1}, P);
end
out(end+1) = "   hand calc: front sprung caster MARGINAL (31.5 N push vs 59 N traction), rear caster PASS, drive wheel 1.84 N.m";
out(end+1) = "   Why the hand calc is optimistic: it gives each drive wheel 42 N. With the preloaded spring the casters carry";
out(end+1) = "   about 27 N between them, so each drive wheel has 29 N and the traction is about 41 N, not 59 N. At a square";
out(end+1) = "   edge the leading caster needs a push of its vertical load x tan(65 deg), about 2.1 x 16 N, and that load grows";
out(end+1) = "   as the carriage compresses, so the drive wheels slip at the edge. More carriage travel alone does not help;";
out(end+1) = "   a softer, lighter spring does, because it leaves more weight on the drive wheels and less on the caster.";
out(end+1) = "   The 3-inch caster lowers the edge angle (tan 53 deg) but with the design spring it still stalls.";
out(end+1) = "";

%% 4. hard stops and launches (dynamic tip-over)
out(end+1) = "4. Hard stops and launches, caster trails turned inward (smallest support polygon)";
hard = {"emergency stop from 0.5 m/s, drawer closed", 0.5, 0, "estop";
        "emergency stop from 0.5 m/s reversing, drawer out 250 mm", -0.5, 0.25, "estop_rev_drawer";
        "full-power launch forward, drawer out 250 mm", 0, 0.25, "launch_drawer"};
for k = 1:size(hard, 1)
    v0 = hard{k,2};
    RV = rv_tunables(P, tip_worst=true, drawer_out=hard{k,3});
    if v0 ~= 0
        ref = prof(t0, [abs(v0)/0.5 v0 0; 1.0 v0 0; 0.02 0 0; 1.5 0 0]);
        tcmd = t0 + abs(v0)/0.5 + 1.0;
    else
        ref = prof(t0, [0.02 1.2 0; 1.5 1.2 0]);    % step to beyond top speed: duty pinned at the cap
        tcmd = t0;
    end
    R = rv_sim(RV, ref, tcmd + 1.5);
    S.(hard{k,4}) = R;
    iw = win(R, tcmd + [0 1.0]);
    amax = max(abs(R.ax(iw)));
    Fmin = min(R.Fn_s(iw,:));
    tl = sum(R.Fn_s(iw,:) < 0.5);                   % ms off the floor (1 ms grid)
    out(end+1) = sprintf("   %s: peak decel/accel %.1f m/s^2, pitch max %.1f deg", hard{k,1}, amax, rad2deg(max(abs(R.pitch(iw))))); %#ok<AGROW>
    out(end+1) = sprintf("      minimum contact loads L %.1f  R %.1f  front %.1f  rear %.1f N", Fmin); %#ok<AGROW>
    out(end+1) = sprintf("      time unloaded (< 0.5 N): L %.0f  R %.0f  front %.0f  rear %.0f ms", tl); %#ok<AGROW>
    plotcase(R, hard{k,4}, hard{k,1}, P);
end
out(end+1) = "   hand calc: forward tip needs 7.6 m/s^2 (0.77 g). A full-reverse stop reaches about the traction limit: the";
out(end+1) = "   rigid rear caster lifts first, the nose settles onto the front carriage stop, and the robot rocks back.";
out(end+1) = "   With the drawer out, a full-power launch (or a hard stop in reverse) unloads the drive wheels: the motor";
out(end+1) = "   reaction pitches the robot back onto the rear caster, which caps the acceleration. Nothing tips over.";
out(end+1) = "";

%% 5. turning at 0.5 m/s with rising yaw rate
ref = prof(t0, [1.0 0.5 0; 6.0 0.5 3.0; 0.5 0.5 3.0]);
R = rv_sim(rv_tunables(P, tip_worst=true), ref, t0 + 7.5);
S.turn = R;
im = R.t > t0 + 1.0;
wz = gradient(unwrap(R.yaw), R.t);
alat = R.vx .* wz;
[~, j] = max(abs(alat(im))); tj = find(im); j = tj(j);
out(end+1) = "5. Turning at 0.5 m/s, yaw rate command ramped 0 to 3 rad/s (left turn)";
out(end+1) = sprintf("   peak lateral acceleration %.2f m/s^2 at yaw rate %.2f rad/s; inner (left) wheel load min %.1f N, outer max %.1f N", ...
    abs(alat(j)), wz(j), min(R.Fn_s(im,1)), max(R.Fn_s(im,2)));
out(end+1) = sprintf("   roll max %.2f deg; static lateral tip needs about %.1f m/s^2 -> %s", rad2deg(max(abs(R.roll(im)))), ...
    P.g * (P.track/2) / P.com(3), pf(min(R.Fn_s(im,1)) > 2, "no wheel lift", "INNER WHEEL LIFTS"));
out(end+1) = sprintf("   outer wheel duty peaks at %.2f (cap %.2f): above ~%.1f rad/s the outer motor runs out of speed", ...
    max(abs(R.d(im,:)), [], "all"), P.driver.pwm_cap, (M.w_free*P.driver.pwm_cap*P.batt.V_full/M.V_rated*P.r_wheel - 0.5)/(P.track/2));
out(end+1) = "";
plotcase(R, "turn", "Turn at 0.5 m/s, rising yaw rate", P);

%% 6. spin in place, aggressive reversal (gearbox side load, recommendation 5)
ref = prof(t0, [0.4 0 3.0; 1.5 0 3.0; 0.15 0 -3.0; 1.5 0 -3.0; 0.4 0 0]);
R = rv_sim(rv_tunables(P), ref, t0 + 4.5);
S.spin = R;
im = R.t > t0;
% joint base frame: y is vertical (down load is negative), z is the axle (lateral)
Frad = hypot(R.fc(:,[1 4]), R.fc(:,[2 5]));
Fax = abs(R.fc(:,[3 6]));
ov = 0.015;                                 % gearbox bushing to wheel centre (hand_calcs.py)
Mb = hypot(Frad*ov, Fax*P.r_wheel);         % bending at the bushing: radial x overhang, lateral tyre force x radius
[mb, j] = max(Mb(im,:), [], "all"); [jj, side] = ind2sub(size(Mb(im,:)), j); tj = find(im); jj = tj(jj);
out(end+1) = "6. Spin in place at 3 rad/s, then reverse to -3 rad/s in 0.15 s";
out(end+1) = sprintf("   peak radial load %.1f N, peak lateral (axial) load %.1f N per wheel", max(Frad(im,:), [], "all"), max(Fax(im,:), [], "all"));
out(end+1) = sprintf("   worst bending at the gearbox bushing %.2f N.m (wheel %s, t = %.2f s): radial %.1f N x 15 mm + lateral %.1f N x %.0f mm", ...
    mb, extractBetween("LR", side, side), R.t(jj), Frad(jj,side), Fax(jj,side), 1000*P.r_wheel);
out(end+1) = sprintf("   compare: straight 2 g bump on the shaft gives %.2f N.m; the lateral tyre force is what loads the gearbox in a spin", ...
    2 * Fs(1) * ov);
out(end+1) = "";
plotcase(R, "spin", "Spin in place with reversal", P);

%% write
f = fullfile(P.dirs.out, "matlab_dynamics.txt");
writelines(out, f);
fprintf("%s\n", out);
fprintf("wrote %s\n", f);
S.report = out;
end

%% ------------------------------------------------------------------ helpers
function ref = prof(t0, seg)
% seg rows: [duration v_end yaw_end], linear ramps; starts at rest after t0
ref = [0 0 0; t0 0 0];
for k = 1:size(seg, 1)
    ref(end+1, :) = [ref(end,1) + seg(k,1), seg(k,2), seg(k,3)]; %#ok<AGROW>
end
ref(end+1, :) = [ref(end,1) + 100, ref(end,2), ref(end,3)];
end

function i = win(R, tw)
i = R.t >= tw(1) & R.t <= tw(2);
end

function [d, ts] = stopdist(R, tb)
i0 = find(R.t >= tb, 1);
k = find(R.t > tb & abs(R.vx) < 0.005, 1);
if isempty(k), k = numel(R.t); end
d = R.x(k) - R.x(i0);
ts = R.t(k) - R.t(i0);
end

function s = row(name, T, I, Th, Ih, Tc)
s = sprintf("   %-22s %5.2f N.m/motor (%3.0f %% of continuous), %4.2f A   | hand calc %4.2f N.m, %4.2f A", ...
    name, T, 100*T/Tc, I, Th, Ih);
end

function s = pf(c, a, b)
if c, s = a; else, s = b; end
end

function plotcase(R, tag, ttl, P)
f = figure(Visible="off", Position=[100 100 900 900]);
tl = tiledlayout(f, 4, 1, TileSpacing="compact");
title(tl, ttl, Interpreter="none");
nexttile; plot(R.t, R.vx, R.t, interp1(R.ref(:,1), R.ref(:,2), R.t), "--");
ylabel("v_x (m/s)"); legend("actual", "command", Location="best"); grid on
nexttile; plot(R.t, R.Tload); yline([P.motor.T_cont -P.motor.T_cont], ":"); ylabel("torque (N.m)"); legend("left", "right", Location="best"); grid on
nexttile; plot(R.t, R.Fn); ylabel("contact load (N)"); legend("left", "right", "front caster", "rear caster", Location="best"); grid on
nexttile; yyaxis left; plot(R.t, rad2deg(R.pitch)); ylabel("pitch (deg)");
yyaxis right; plot(R.t, 1000*R.q_spring); ylabel("carriage (mm)"); xlabel("time (s)"); grid on
exportgraphics(f, fullfile(P.dirs.out, "matlab_dyn_" + tag + ".png"), Resolution=110);
close(f);
end
