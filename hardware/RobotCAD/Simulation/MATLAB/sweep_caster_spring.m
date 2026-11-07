function T = sweep_caster_spring(P, k_per_spring, F0_per_spring)
%SWEEP_CASTER_SPRING  Front caster spring rate and preload against braking and thresholds.
%   T = sweep_caster_spring() simulates, for each combination (two springs;
%   values are per spring, carriage travel as designed):
%     braking   1 m/s^2 stop from 0.5 m/s: carriage dip, rear caster load, pitch
%     threshold 15 mm square edge at 0.3 m/s, forward and in reverse
%   and picks the stiffest, most preloaded spring that still crosses both ways
%   with at least 4 mm of bump travel left and the rear caster on the floor.
%   Writes results/matlab_caster_spring.txt/.csv and a map of the grid.

arguments
    P struct = rv_params()
    k_per_spring double = [0.25 0.5 1.0 1.5]   % N/mm
    F0_per_spring double = [0 3 6]              % N
end
t0 = 0.5;
ref_b = [0 0 0; t0 0 0; t0+1 0.5 0; t0+1.5 0.5 0; t0+2.0 0 0; t0+100 0 0];
tb = t0 + 1.5;
rows = [];
for k = k_per_spring
    for F0 = F0_per_spring
        base = {"k_spring", 2*k*1e3, "F0_spring", 2*F0};
        R = rv_sim(rv_tunables(P, base{:}), ref_b, tb + 1.0);
        is = R.t > t0 - 0.15 & R.t < t0;
        ib = R.t >= tb & R.t <= tb + 0.8;
        Nf = mean(R.Fn(is,3)); Nr = mean(R.Fn(is,4)); Nw = mean(R.Fn(is,1));
        dip = max(R.q_spring(ib));
        rmin = min(R.Fn_s(ib,4));
        pitch = rad2deg(max(R.pitch(ib)) - mean(R.pitch(is)));
        cross = [0 0];
        for d = [1 -1]
            x_edge = d * 0.35; if d < 0, x_edge = x_edge - 0.04; end
            Rt = rv_sim(rv_tunables(P, base{:}, x_thr=x_edge, direction=d), ...
                [0 0 0; t0 0 0; t0+0.6 0.3*d 0; t0+100 0.3*d 0], 5.0);
            goal = abs(x_edge) + 0.04 + P.caster_x + P.caster_trail + 0.06;
            cross((3 - d)/2) = abs(Rt.x(end)) > goal;
        end
        rows(end+1, :) = [k F0 Nw Nf Nr 1000*dip 1000*(P.spring_travel - dip) rmin pitch cross]; %#ok<AGROW>
        fprintf("k %.2f N/mm F0 %4.1f N: wheels %.1f front %.1f rear %.1f N, dip %.1f mm, rear min %.1f N, threshold fwd %d rev %d\n", ...
            k, F0, Nw, Nf, Nr, 1000*dip, rmin, cross);
    end
end
T = array2table(rows, VariableNames=["k_N_per_mm" "preload_N" "wheel_static_N" "front_static_N" "rear_static_N" ...
    "q_brake_max_mm" "bump_left_mm" "rear_min_brake_N" "pitch_brake_deg" "crosses_fwd" "crosses_rev"]);
writetable(T, fullfile(P.dirs.out, "matlab_caster_spring.csv"));

ok = T.crosses_fwd & T.crosses_rev & T.bump_left_mm >= 4 & T.rear_min_brake_N >= 0.5;
out = strings(0);
out(end+1) = "Front caster spring sweep (two springs; rate and preload are per spring; carriage travel 10 mm as designed)";
out(end+1) = "Braking: 1 m/s^2 from 0.5 m/s. Threshold: 15 mm square edge at 0.3 m/s, forward and reverse.";
out(end+1) = "Pass: crosses both ways, >= 4 mm bump travel left while braking, rear caster stays on the floor.";
out(end+1) = "";
out(end+1) = "  k N/mm  preload N  wheel N  front N  rear N  dip mm  bump left mm  rear min N  pitch deg  fwd  rev  pass";
yn = ["no" "yes"];
for i = 1:height(T)
    out(end+1) = sprintf("  %5.2f  %8.1f  %7.1f  %7.1f  %6.1f  %6.1f  %12.1f  %10.1f  %9.2f  %3s  %3s  %s", T{i,1:9}, ...
        yn(T.crosses_fwd(i)+1), yn(T.crosses_rev(i)+1), yn(ok(i)+1)); %#ok<AGROW>
end
out(end+1) = "";
cur = abs(T.k_N_per_mm - 1.5) < 1e-9 & T.preload_N == 6;
if any(cur)
    why = ["threshold forward" "threshold reverse" "bump travel" "rear caster lifts"];
    bad = ~[T.crosses_fwd(cur) T.crosses_rev(cur) T.bump_left_mm(cur) >= 4 T.rear_min_brake_N(cur) >= 0.5];
    out(end+1) = sprintf("Current recommendation (1.5 N/mm, 6 N each): %s.", ...
        pf(ok(cur), "passes", "fails (" + strjoin(why(bad), ", ") + ")"));
end
if any(ok)
    Tk = T(ok, :);
    [~, j] = max(Tk.preload_N * 1e3 + Tk.k_N_per_mm);
    out(end+1) = sprintf("Pick: %.2f N/mm with %.0f N preload per spring (front %.1f N standing, dip %.1f mm braking, rear min %.1f N).", ...
        Tk.k_N_per_mm(j), Tk.preload_N(j), Tk.front_static_N(j), Tk.q_brake_max_mm(j), Tk.rear_min_brake_N(j));
    out(end+1) = "This is the firmest spring in the grid that still gets over a square 15 mm edge both ways. Choose a stock";
    out(end+1) = "compression spring near that rate whose installed length gives that preload, and check the Lee Spring";
    out(end+1) = "LC 032E 05 S from the README shortlist against its datasheet before ordering.";
else
    out(end+1) = "No combination in the grid passes.";
end
fn = fullfile(P.dirs.out, "matlab_caster_spring.txt");
writelines(out, fn); fprintf("%s\n", out);

f = figure(Visible="off", Position=[100 100 1000 420]);
tiledlayout(f, 1, 2);
nk = numel(k_per_spring); nf = numel(F0_per_spring);
nexttile; imagesc(k_per_spring, F0_per_spring, reshape(double(ok), nf, nk)); axis xy; colormap(gca, [0.85 0.4 0.4; 0.45 0.75 0.45]);
xlabel("rate per spring (N/mm)"); ylabel("preload per spring (N)"); title("passes (green) / fails (red)");
nexttile; imagesc(k_per_spring, F0_per_spring, reshape(T.q_brake_max_mm, nf, nk)); axis xy; colorbar;
xlabel("rate per spring (N/mm)"); ylabel("preload per spring (N)"); title("carriage dip braking 1 m/s^2 (mm)");
exportgraphics(f, fullfile(P.dirs.out, "matlab_caster_spring.png"), Resolution=110); close(f);
end

function s = pf(c, a, b)
if c, s = a; else, s = b; end
end
