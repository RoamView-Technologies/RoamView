function run_all(opts)
%RUN_ALL  Rebuild the RoamView model and run every MATLAB simulation.
%   From MATLAB, in this folder:   run_all
%   From a terminal:               matlab -batch "run_all"
%   Reports and plots land in results/. A full run takes about five minutes.
%
%   run_all(animate=true) shows every multibody run in Mechanics Explorer at
%   real time (about ten minutes); add pace=0.5 for half speed, or
%   sweep=false to skip the 36 runs of the spring sweep. The results are the
%   same as without animation.

arguments
    opts.animate (1,1) logical = false
    opts.pace (1,1) double {mustBePositive} = 1
    opts.sweep (1,1) logical = true
end
if opts.animate
    setappdata(0, "rv_animate", opts.pace);
    cleanup = onCleanup(@() rmappdata(0, "rv_animate"));
    if ~usejava("desktop"), warning("run_all:nodesktop", "No MATLAB desktop, so nothing is animated."); end
end

P = rv_params();
fprintf("RobotCAD data: %s\n", P.root);
fprintf("Robot %.2f kg, COM (%.1f, %.1f, %.1f) mm\n\n", P.m_total, 1000*P.com);

build_rv_dynamics(P);
steps = {"dynamics", @() run_dynamics(P);
         "drivetrain electrical", @() run_drivetrain_electrical(P);
         "caster spring sweep", @() sweep_caster_spring(P);
         "FEA cross-check", @() fea_pde("all", P)};
if ~opts.sweep, steps(3, :) = []; end
status = strings(size(steps, 1), 1);
for k = 1:size(steps, 1)
    fprintf("\n===== %s =====\n", steps{k,1});
    t = tic;
    try
        steps{k,2}();
        status(k) = sprintf("done in %.0f s", toc(t));
    catch err
        status(k) = "FAILED: " + err.message;
        fprintf(2, "%s\n", getReport(err));
    end
end
fprintf("\n===== summary =====\n");
for k = 1:size(steps, 1), fprintf("  %-22s %s\n", steps{k,1}, status(k)); end
fprintf("results in %s\n", P.dirs.out);
end
