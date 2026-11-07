function R = run_arena(opts)
%RUN_ARENA  Drive RoamView through the two-room test course and check the design.
%   run_arena                        current front spring, animated if MATLAB has a desktop
%   run_arena(spring="recommended")  0.5 N/mm with 6 N preload per spring
%   run_arena(video=true)            also renders results/matlab_arena_<spring>.mp4
%   run_arena(animate=false)         no Mechanics Explorer (faster)
%   run_arena(result=R, video=true)  redo the report, plots and video from an earlier run
%
%   The course (rv_arena_layout.m): a 5 degree ramp up to a 50 mm platform and
%   down, a 12 mm cable cover, a slalom through table legs, a sofa and a bin to
%   pass, and a 0.8 m doorway with a 15 mm threshold into the second room. The
%   robot steers itself along the route (rv_pursuit.m) at up to 0.4 m/s.
%   It reports whether the robot finishes, where it gets stuck, collisions of
%   its footprint with walls and furniture, scraping of its underside, the
%   closest it comes to each obstacle, tilt, wheel and caster loads, motor
%   current and battery energy, section by section. The physics is the same
%   model as the validation runs; the arena takes about ten times real time to
%   compute (some 20 minutes).

arguments
    opts.spring (1,1) string {mustBeMember(opts.spring, ["design" "recommended"])} = "design"
    opts.animate (1,1) logical = usejava("desktop")
    opts.pace (1,1) double {mustBePositive} = 1
    opts.video (1,1) logical = false
    opts.rebuild (1,1) logical = false
    opts.result struct = struct([])
end
P = rv_params();
A = rv_arena_layout(P);
if isempty(opts.result)
    here = fileparts(mfilename("fullpath"));
    if opts.rebuild || ~isfile(fullfile(here, "rv_arena.slx")), build_rv_dynamics(P, "cad", "arena"); end
    sp = {};
    if opts.spring == "recommended", sp = {"k_spring", 2*0.5e3, "F0_spring", 2*6}; end
    RV = rv_tunables(P, sp{:}, scene="arena", start=A.start);
    rv_pursuit("init", A.path, struct(speed=A.speed, lookahead=A.lookahead, t0=0.5));
    fprintf("RoamView arena run, %s spring: %.2f N/mm with %.0f N preload per spring\n", opts.spring, RV.k_spring/2e3, RV.F0_spring/2);
    tic;
    R = rv_sim(RV, [0 0 0; 1 0 0], A.tstop, opts.animate * opts.pace);
    fprintf("simulated %.0f s in %.0f s\n", A.tstop, toc);
    S = rv_pursuit("state");
    R.route_done = S.done;
    R.route_s = S.s(S.idx) / S.s(end);
    R.spring = opts.spring;
else
    R = opts.result;                          % redo the report and video from an earlier run
end

[txt, M] = arena_report(R, A, P);
tag = "matlab_arena_" + R.spring;
writelines(txt, fullfile(P.dirs.out, tag + ".txt"));
fprintf("%s\n", txt);
arena_plots(R, A, M, fullfile(P.dirs.out, tag + ".png"));
R.metrics = M;
if opts.video
    rv_arena_video(R, A, P, fullfile(P.dirs.out, tag + ".mp4"));
end
end

%% ---------------------------------------------------------------- report
function [out, M] = arena_report(R, A, P)
t = R.t;
xy = [R.xw R.yw];
dist = sum(vecnorm(diff(xy), 2, 2));
fin = find(vecnorm(xy - A.waypoints(end,:), 2, 2) < 0.06, 1);
M.finished = ~isempty(fin);
Pbat = R.Vbatt .* sum(R.d .* R.i, 2);                   % battery power into the two motor channels
M.energy_Wh = trapz(t, max(Pbat, 0)) / 3600;
[M.clear, M.clear_obj, M.clear_t] = clearance(R, A);
[M.fcoll, ic] = max(R.Fcoll);
[M.fscrape, is] = max(R.Fscrape);

out = strings(0);
out(end+1) = "RoamView arena run (rv_arena.slx, Simscape Multibody), " + string(datetime("now", Format="yyyy-MM-dd HH:mm"));
out(end+1) = sprintf("Robot %.2f kg; front spring %.2f N/mm with %.0f N preload per spring (%s); cruise %.1f m/s, pure pursuit", ...
    P.m_total, R.RV.k_spring/2e3, R.RV.F0_spring/2, R.spring, A.speed);
out(end+1) = sprintf("Footprint for the checks: x %.0f to %.0f mm, width %.0f mm over the tyres, underside %.0f mm above the floor", ...
    1000*A.box(1), 1000*A.box(2), 2000*A.box(3), 1000*A.box(4));
out(end+1) = "";
if M.finished
    out(end+1) = sprintf("RESULT: finished the course in %.1f s (%.1f m driven).", t(fin), dist);
else
    [~, km] = min(vecnorm(reshape([A.marks.xy], 2, []).' - xy(end,:), 2, 2));
    out(end+1) = sprintf("RESULT: did NOT finish. Stopped at x %.2f y %.2f, %.0f %% along the route, next to the %s, after %.1f m.", ...
        xy(end,1), xy(end,2), 100*R.route_s, A.marks(km).name, dist);
end
out(end+1) = sprintf("Collisions with walls or furniture: %s", pf(M.fcoll < 1, "none", ...
    sprintf("peak %.0f N at t %.1f s, x %.2f y %.2f", M.fcoll, t(ic), xy(ic,1), xy(ic,2))));
out(end+1) = sprintf("Underside scraping on ramp, cable cover or threshold: %s", pf(M.fscrape < 1, "none", ...
    sprintf("peak %.0f N at t %.1f s, x %.2f y %.2f", M.fscrape, t(is), xy(is,1), xy(is,2))));
out(end+1) = sprintf("Closest approach to an obstacle: %.0f mm (%s, t %.1f s)", 1000*M.clear, M.clear_obj, M.clear_t);
out(end+1) = sprintf("Max pitch %.1f deg, max roll %.1f deg; peak motor current %.1f A; battery energy %.2f Wh for the motors", ...
    rad2deg(max(abs(R.pitch))), rad2deg(max(abs(R.roll))), max(abs(R.i), [], "all"), M.energy_Wh);
out(end+1) = "";
out(end+1) = "Section by section (while the robot centre is within 0.5 m of each feature):";
out(end+1) = "  feature              reached  time s   pitch deg  roll deg  min wheel N  min front N  min rear N  peak A  front spring mm";
for m = A.marks
    in = vecnorm(xy - m.xy, 2, 2) < 0.5 & t > 0.5;
    if ~any(in)
        out(end+1) = sprintf("  %-20s %7s", m.name, "no"); %#ok<AGROW>
        continue
    end
    ti = t(find(in, 1));
    out(end+1) = sprintf("  %-20s %7s  %6.1f  %9.1f  %8.1f  %11.1f  %11.1f  %10.1f  %6.1f  %15.1f", m.name, "yes", ti, ...
        rad2deg(max(abs(R.pitch(in)))), rad2deg(max(abs(R.roll(in)))), min(R.Fn_s(in,1:2), [], "all"), ...
        min(R.Fn_s(in,3)), min(R.Fn_s(in,4)), max(abs(R.i(in,:)), [], "all"), 1000*max(R.q_spring(in))); %#ok<AGROW>
end
out(end+1) = "";
out(end+1) = "A load of 0 N means that wheel or caster left the floor. The obstacles are rigid; the collision check uses a";
out(end+1) = "box over the robot's plan extents (tyres and charge pads included) from the lowest chassis point to the deck.";
end

function [cmin, obj, tmin] = clearance(R, A)
% smallest plan distance between the footprint box and any obstacle, sampled every 50 ms
B = A.box;
[gx, gy] = meshgrid(linspace(B(1), B(2), 15), linspace(-B(3), B(3), 9));
edge = [gx(:) gy(:)];
edge = edge(abs(edge(:,1) - B(1)) < 1e-9 | abs(edge(:,1) - B(2)) < 1e-9 | abs(abs(edge(:,2)) - B(3)) < 1e-9, :);
O = A.objects([A.objects.role] == "obstacle");
cmin = inf; obj = ""; tmin = 0;
for k = 1:50:numel(R.t)
    c = cos(R.yaw(k)); s = sin(R.yaw(k));
    pts = edge * [c s; -s c] + [R.xw(k) R.yw(k)];
    for o = O
        if o.shape == "cyl"
            d = min(vecnorm(pts - o.pos(1:2), 2, 2)) - o.dims(1);
        else
            h = o.dims(1:2) / 2;
            q = max(abs(pts - o.pos(1:2)) - h, 0);
            d = min(vecnorm(q, 2, 2));
        end
        if d < cmin, cmin = d; obj = o.name; tmin = R.t(k); end
    end
end
end

%% ---------------------------------------------------------------- plots
function arena_plots(R, A, M, fn)
f = figure(Visible="off", Position=[50 50 1500 950]);
tl = tiledlayout(f, 3, 3, TileSpacing="compact");
title(tl, sprintf("RoamView arena run, %s spring: %s", R.spring, pf(M.finished, "finished", "did not finish")));
ax = nexttile(tl, [2 2]); hold(ax, "on"); axis(ax, "equal"); grid(ax, "on");
draw_plan(ax, A);
plot(ax, A.path(:,1), A.path(:,2), "k--", LineWidth=0.8);
scatter(ax, R.xw(1:20:end), R.yw(1:20:end), 6, R.vx(1:20:end), "filled");
cb = colorbar(ax); cb.Label.String = "speed (m/s)";
B = A.box;
for k = 1:3000:numel(R.t)
    c = cos(R.yaw(k)); s = sin(R.yaw(k));
    pts = [B(1) -B(3); B(2) -B(3); B(2) B(3); B(1) B(3); B(1) -B(3)] * [c s; -s c] + [R.xw(k) R.yw(k)];
    plot(ax, pts(:,1), pts(:,2), Color=[0.2 0.3 0.8 0.6]);
end
hit = R.Fcoll > 1 | R.Fscrape > 1;
if any(hit), plot(ax, R.xw(hit), R.yw(hit), "rx", MarkerSize=8, LineWidth=1.5); end
if ~M.finished, plot(ax, R.xw(end), R.yw(end), "rp", MarkerSize=18, MarkerFaceColor="r"); end
for m = A.marks, text(ax, m.xy(1), m.xy(2) + 0.32, m.name, HorizontalAlignment="center", FontSize=8); end
xlabel(ax, "x (m)"); ylabel(ax, "y (m)"); title(ax, "route (dashed), driven path coloured by speed, footprint every 3 s");

nexttile(tl); plot(R.t, R.vcmd, "k--", R.t, R.vx); grid on; ylabel("speed (m/s)"); legend("command", "actual", Location="best");
nexttile(tl); plot(R.t, rad2deg(R.pitch), R.t, rad2deg(R.roll)); grid on; ylabel("deg"); legend("pitch", "roll", Location="best");
nexttile(tl); plot(R.t, R.Fn_s); grid on; ylabel("contact load (N)"); xlabel("time (s)");
legend("left wheel", "right wheel", "front caster", "rear caster", Location="best");
nexttile(tl); plot(R.t, R.i); grid on; ylabel("motor current (A)"); xlabel("time (s)"); legend("left", "right", Location="best");
nexttile(tl); plot(R.t, R.Fcoll, R.t, R.Fscrape, R.t, 1000*R.q_spring); grid on; xlabel("time (s)");
legend("collision force (N)", "scrape force (N)", "front spring (mm)", Location="best");
exportgraphics(f, fn, Resolution=100);
close(f);
end

function draw_plan(ax, A)
for o = A.objects
    if o.name == "floor", continue; end
    if o.shape == "cyl" && o.axis == "+Z"
        th = linspace(0, 2*pi, 40);
        fill(ax, o.pos(1) + o.dims(1)*cos(th), o.pos(2) + o.dims(1)*sin(th), o.color, EdgeColor="k");
    else
        d = o.dims;
        if o.shape == "cyl", d = [2*o.dims(1) o.dims(2)]; end          % lying cylinder: axis along y
        if o.axis == "+Y", d(1) = d(1) * cosd(o.angle); end
        rectangle(ax, Position=[o.pos(1) - d(1)/2, o.pos(2) - d(2)/2, d(1), d(2)], FaceColor=o.color, EdgeColor="k");
    end
end
end

function s = pf(c, a, b)
if c, s = a; else, s = b; end
end
