function E = run_drivetrain_electrical(P)
%RUN_DRIVETRAIN_ELECTRICAL  Motor, driver and battery checks for RoamView.
%   Uses the averaged PWM + DC motor model that drives rv_dynamics.slx:
%     i = (d*Vbatt - Ke*w)/R,  T = Kt*i - T_fric   (all at the gearbox output)
%   Writes results/matlab_drivetrain_electrical.txt and a torque-speed plot.
%   Battery capacity and the electronics load are placeholders in rv_params.m.

if nargin < 1, P = rv_params(); end
M = P.motor; B = P.batt; cap = P.driver.pwm_cap;
out = strings(0);
out(end+1) = "RoamView drivetrain electrical checks (averaged PWM, DC motor referred to the gearbox output)";
out(end+1) = sprintf("Motor model from the 12 V datasheet: R %.2f ohm, Kt %.3f N.m/A, Ke %.3f V.s/rad, no-load loss %.2f N.m", ...
    M.R, M.Kt, M.Ke, M.T_fric);
out(end+1) = "";

%% 1. stall current and torque against supply and duty limit
out(end+1) = "1. Stall (wheel blocked), per motor";
cases = {"12 V datasheet, full duty", M.V_rated, 1;
         "4S full 16.8 V, full duty", B.V_full, 1;
         "4S full 16.8 V, duty capped", B.V_full, cap;
         "4S nominal 14.8 V, duty capped", B.V_nom, cap;
         "4S empty 12.8 V, duty capped", B.V_empty, cap};
for k = 1:size(cases, 1)
    V = cases{k,2} * cases{k,3};
    Is = V / M.R; Ts = M.Kt * Is - M.T_fric; wf = (V - M.I_free*M.R) / M.Ke;
    out(end+1) = sprintf("   %-32s %5.2f V at the motor: stall %4.1f A, %4.2f N.m; top speed %.2f m/s %s", ...
        cases{k,1}, V, Is, Ts, wf * P.r_wheel, ...
        pf(Is <= M.I_stall * 1.02, "", sprintf("(%.0f %% over the 12 V stall current)", 100*(Is/M.I_stall - 1)))); %#ok<AGROW>
end
out(end+1) = sprintf("   both motors stalled at full charge, uncapped: %.1f A from the pack; capped: %.1f A (DFR0601 %d A/ch)", ...
    2*B.V_full/M.R, 2*cap*B.V_full/M.R, P.driver.I_max);
out(end+1) = "";

%% 2. dynamic runs: threshold climb at the corners of the supply envelope
out(end+1) = "2. 15 mm threshold at 0.3 m/s, forward (front sprung caster leads)";
t0 = 0.5;
ref = [0 0 0; t0 0 0; t0+0.6 0.3 0; t0+100 0.3 0];
runs = {"full 16.8 V, capped", B.V_full, cap; "full 16.8 V, uncapped", B.V_full, 1; "empty 12.8 V, capped", B.V_empty, cap};
for k = 1:size(runs, 1)
    R = rv_sim(rv_tunables(P, x_thr=0.35, Vbatt=runs{k,2}, pwm_cap=runs{k,3}), ref, 5.0);
    im = R.t > t0 + 0.8;
    goal = 0.35 + 0.04 + P.caster_x + P.caster_trail + 0.06;
    Pbus = sum(R.d .* R.i, 2) .* R.Vbatt;
    out(end+1) = sprintf("   %-22s %s; peak current %.1f A, peak torque %.2f N.m, peak pack draw %.0f W (motors)", ...
        runs{k,1}, pf(R.x(end) > goal, "crosses", "STUCK"), max(abs(R.i(im,:)), [], "all"), ...
        max(abs(R.T(im,:)), [], "all"), max(Pbus(im))); %#ok<AGROW>
end
out(end+1) = "";

%% 3. runtime estimate for a mission profile
out(end+1) = "3. Runtime estimate (PLACEHOLDER pack and electronics load, see rv_params.m)";
R = rv_sim(rv_tunables(P, Vbatt=B.V_nom), [0 0 0; t0 0 0; t0+1 0.5 0; t0+4 0.5 0; t0+100 0.5 0], 4.5);
ic = R.t > t0 + 1.8;
P_cruise = mean(sum(R.d(ic,:) .* R.i(ic,:), 2) .* R.Vbatt(ic));
R2 = rv_sim(rv_tunables(P, Vbatt=B.V_nom, ramp_deg=5), [0 0 0; t0 0 0; t0+0.6 0.3 0; t0+100 0.3 0], 3.5);
ic = R2.t > t0 + 1.5;
P_ramp = mean(sum(R2.d(ic,:) .* R2.i(ic,:), 2) .* R2.Vbatt(ic));
P_idle = 2 * 0 + 0;                     % motors off when parked (driver standby ignored)
mix = [0.35 0.05 0.60];                 % share of time: cruising 0.5 m/s, climbing 5 deg ramps, parked
Pmot = mix * [P_cruise; P_ramp; P_idle];
Ptot = Pmot + P.elec_load_W;
Ewh = B.capacity_Ah * B.V_nom * 0.85;   % 85 % usable
out(end+1) = sprintf("   motor bus power: cruise 0.5 m/s %.1f W, 5 deg ramp at 0.3 m/s %.1f W", P_cruise, P_ramp);
out(end+1) = sprintf("   mission mix %.0f %% cruise / %.0f %% ramp / %.0f %% parked -> motors %.1f W + electronics %.0f W = %.1f W", ...
    100*mix, Pmot, P.elec_load_W, Ptot);
out(end+1) = sprintf("   %.1f Ah 4S pack, 85 %% usable (%.0f Wh): about %.1f h; the electronics take %.0f %% of the energy", ...
    B.capacity_Ah, Ewh, Ewh / Ptot, 100 * P.elec_load_W / Ptot);
out(end+1) = "";

%% torque-speed plot
f = figure(Visible="off", Position=[100 100 800 520]); hold on
for k = [1 2 3 5]
    V = cases{k,2} * cases{k,3};
    w = linspace(0, (V - M.I_free*M.R)/M.Ke, 50);
    plot(w * P.r_wheel, M.Kt * (V - M.Ke*w)/M.R - M.T_fric, DisplayName=cases{k,1});
end
yline(M.T_cont, ":", "continuous limit"); yline(M.T_inst, ":", "instantaneous limit");
v = [0.5 0.3]; T = [P_cruise P_ramp];  %#ok<NASGU>
xlabel("ground speed (m/s)"); ylabel("torque per motor at the output (N.m)"); grid on; legend(Location="northeast");
title("Pololu 37D 70:1 on a 4S pack");
exportgraphics(f, fullfile(P.dirs.out, "matlab_torque_speed.png"), Resolution=110); close(f);

fn = fullfile(P.dirs.out, "matlab_drivetrain_electrical.txt");
writelines(out, fn); fprintf("%s\n", out); fprintf("wrote %s\n", fn);
E.report = out;
end

function s = pf(c, a, b)
if c, s = a; else, s = b; end
end
