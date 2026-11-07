function R = watch_case(name, varargin)
%WATCH_CASE  Run one RoamView case and animate it in Mechanics Explorer.
%   watch_case("threshold")             15 mm door threshold, current spring (stalls)
%   watch_case("threshold_new_spring")  same edge with 0.5 N/mm and 6 N per spring (crosses)
%   watch_case list                     prints every case
%   Any rv_tunables option can follow and overrides the case, for example
%   watch_case("threshold", caster_d=0.075) or watch_case("ramp", ramp_deg=10).
%   R = watch_case(...) also returns the signals (see rv_sim) and a figure of
%   wheel speed, contact loads and spring compression opens next to the animation.
%   In Mechanics Explorer, the playback bar replays the run; the speed box sets
%   slow motion, and the view buttons switch between front, side and top views.

arguments
    name (1,1) string = "threshold"
end
arguments (Repeating)
    varargin
end

t0 = 0.5;
%            name                    tunables                                          [duration v_end yaw_end] segments           stop s  what to look at
C = {"static",               {},                                                        zeros(0,3),                                    2.0, "robot settles on its four contacts";
     "flat",                 {},                                                        [1.0 0.5 0; 2.0 0.5 0; 0.5 0 0; 1.5 0 0],     6.0, "accelerate to 0.5 m/s, cruise, brake at 1 m/s^2";
     "ramp",                 {"ramp_deg", 5},                                           [0.6 0.3 0; 2.5 0.3 0],                        4.0, "climb a 5 degree ramp";
     "threshold",            {"x_thr", 0.35},                                           [0.6 0.3 0; 4.0 0.3 0],                        5.0, "15 mm edge, current spring: the caster stalls at the edge";
     "threshold_reverse",    {"x_thr", -0.39, "direction", -1},                         [0.6 -0.3 0; 4.0 -0.3 0],                      5.0, "15 mm edge in reverse, rear caster leads";
     "threshold_new_spring", {"x_thr", 0.35, "k_spring", 1000, "F0_spring", 12},        [0.6 0.3 0; 4.0 0.3 0],                        5.0, "15 mm edge with 0.5 N/mm, 6 N per spring: crosses";
     "estop",                {"tip_worst", true},                                       [1.0 0.5 0; 1.0 0.5 0; 0.02 0 0; 1.5 0 0],     4.0, "full-reverse emergency stop from 0.5 m/s: rear caster lifts";
     "launch_drawer",        {"tip_worst", true, "drawer_out", 0.25},                   [0.02 1.2 0; 1.5 1.2 0],                       2.0, "full-power launch with the drawer out 250 mm";
     "turn",                 {"tip_worst", true},                                       [1.0 0.5 0; 6.0 0.5 3.0; 0.5 0.5 3.0],         8.0, "0.5 m/s with the turn rate rising to 3 rad/s";
     "spin",                 {},                                                        [0.4 0 3.0; 1.5 0 3.0; 0.15 0 -3.0; 1.5 0 -3.0; 0.4 0 0], 5.0, "spin in place with a fast reversal"};

if name == "list"
    for k = 1:size(C, 1), fprintf("  %-22s %s\n", C{k,1}, C{k,5}); end
    if nargout, R = []; end
    return
end
k = find(name == string(C(:,1)));
if isempty(k)
    error("watch_case:unknown", "Unknown case %s. Cases: %s", name, strjoin(string(C(:,1)), ", "));
end

P = rv_params();
here = fileparts(mfilename("fullpath"));
if ~isfile(fullfile(here, "rv_dynamics.slx")), build_rv_dynamics(P); end
RV = rv_tunables(P, C{k,2}{:}, varargin{:});
ref = [0 0 0; t0 0 0];
seg = C{k,3};
for j = 1:size(seg, 1), ref(end+1, :) = [ref(end,1) + seg(j,1), seg(j,2:3)]; end %#ok<AGROW>
ref(end+1, :) = [ref(end,1) + 100, ref(end,2:3)];

fprintf("%s: %s\n", C{k,1}, C{k,5});
show = usejava("desktop");                 % no window to draw in under matlab -batch
Rk = rv_sim(RV, ref, C{k,4}, show);
fprintf("  travelled %.2f m, peak pitch %.1f deg, front spring up to %.1f mm, min load L %.1f R %.1f front %.1f rear %.1f N\n", ...
    Rk.x(end), rad2deg(max(abs(Rk.pitch))), 1000*max(Rk.q_spring), min(Rk.Fn_s(Rk.t > t0, :)));

if show
    f = figure(Name="RoamView " + C{k,1}, NumberTitle="off");
    tiledlayout(f, 3, 1, TileSpacing="compact");
    nexttile; plot(Rk.t, Rk.w * P.r_wheel, Rk.t, Rk.wref(:,1) * P.r_wheel, "k--"); grid on
    ylabel("wheel speed (m/s)"); legend("left", "right", "command", Location="best"); title(C{k,5});
    nexttile; plot(Rk.t, Rk.Fn_s); grid on; ylabel("contact load (N)");
    legend("left wheel", "right wheel", "front caster", "rear caster", Location="best");
    nexttile; plot(Rk.t, 1000 * Rk.q_spring); grid on; ylabel("front spring (mm)"); xlabel("time (s)");
end
if nargout, R = Rk; end
end
