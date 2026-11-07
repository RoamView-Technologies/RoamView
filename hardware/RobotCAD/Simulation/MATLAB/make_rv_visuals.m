function V = make_rv_visuals(P, force)
%MAKE_RV_VISUALS  Turn the SolidWorks part STLs into the meshes the model animates.
%   V = make_rv_visuals(P) reads Simulation/FOV/parts (one STL per component in
%   the RV-100 assembly frame, written by export_fov_stl.py), groups the parts
%   by the body they ride on, thins each mesh and writes visuals/*.stl in metres:
%     chassis_*   everything fixed to the chassis, ROS frame (floor under the axle)
%     drawer      battery drawer, ROS frame; the model slides it by RV.drawer_out
%     carriage    front caster carriage and caster, ROS frame; the model moves it
%                 with the spring
%     wheel*_*    tyre and hub, in the wheel joint frame (z along the axle)
%   V lists each mesh with its body, file, colour and opacity. Existing meshes are
%   reused unless force is true. The meshes are for looks only; mass and
%   inertia come from rv_params.

arguments
    P struct = rv_params()
    force (1,1) logical = false
end
here = fileparts(mfilename("fullpath"));
outd = fullfile(here, "visuals");
src = fullfile(P.dirs.sim, "FOV", "parts");

% body, mesh name, colour, opacity, regular expression matched against the STL file names
metal = [0.62 0.64 0.67]; shell = [0.93 0.93 0.9]; dark = [0.22 0.22 0.24]; tyre = [0.08 0.08 0.08];
groups = {
    "wheel",    "tyre",          tyre,  1,    "RV-503_";
    "wheel",    "hub",           metal, 1,    "RV-502_";
    "carriage", "carriage",      [0.9 0.6 0.1], 1, "RV-120_.*(RV-309_|RV-504_)";
    "drawer",   "drawer",        [0.35 0.5 0.75], 1, "RV-130_";
    "chassis",  "chassis_shell", shell, 0.35, "RV-30[12]_|RV-31[345]_";
    "chassis",  "chassis_metal", metal, 1,    "RV-4(10|20|30|31|33|34|35)_|RV-501_";
    "chassis",  "chassis_dark",  dark,  1,    ""};          % everything left over

V = struct("body", {}, "name", {}, "file", {}, "color", {}, "opacity", {});
for g = 1:size(groups, 1)
    if groups{g,1} == "wheel"
        for side = ["L" "R"]
            V(end+1) = mk("wheel" + side, "wheel" + side + "_" + groups{g,2}, groups(g,:)); %#ok<AGROW>
        end
    else
        V(end+1) = mk(groups{g,1}, groups{g,2}, groups(g,:)); %#ok<AGROW>
    end
end
if ~force && all(arrayfun(@(v) isfile(v.file), V)), return; end
if ~isfolder(src)
    error("make_rv_visuals:noparts", "No part STLs in %s. Run export_fov_stl.py with SolidWorks open.", src);
end
if ~isfolder(outd), mkdir(outd); end

files = dir(fullfile(src, "*.STL"));
names = string({files.name});
used = false(size(names));
acc = containers.Map();
for g = 1:size(groups, 1)
    pat = groups{g,5};
    if pat == "", hit = ~used; else, hit = ~used & ~cellfun(@isempty, regexp(names, pat, "once")); end
    used = used | hit;
    for f = find(hit)
        [Fc, Vr] = readpart(fullfile(src, names(f)));
        key = groups{g,2};
        if groups{g,1} == "wheel"
            sideL = mean(Vr(:,2)) > 0;
            sides = ["R" "L"]; key = "wheel" + sides(sideL + 1) + "_" + key;
            c = [0, (2*sideL - 1) * P.track/2, P.r_wheel];
            Rw = [1 0 0; 0 0 1; 0 -1 0];                 % wheel_T: -90 deg about x, so frame z = chassis +y
            Vr = (Vr - c) * Rw;                          % row vectors: p_frame = Rw' * (p - c)
        end
        if isKey(acc, key), m = acc(key); else, m = {zeros(0,3), zeros(0,3)}; end
        m{1} = [m{1}; Fc + size(m{2}, 1)]; m{2} = [m{2}; Vr];
        acc(key) = m;
    end
end
for v = V
    if ~isKey(acc, v.name), error("make_rv_visuals:empty", "No parts found for %s.", v.name); end
    m = acc(v.name);
    stlwrite(triangulation(m{1}, m{2}), v.file);
    fprintf("  %-20s %7d triangles\n", v.name, size(m{1}, 1));
end

    function v = mk(body, name, g)
        v = struct("body", body, "name", name, "file", fullfile(outd, name + ".stl"), ...
            "color", g{3}, "opacity", g{4});
    end
end

function [F, V] = readpart(fn)
% SolidWorks assembly frame in mm (X left, Y up, Z forward) -> ROS metres, then thin it
T = stlread(fn);
V = T.Points(:, [3 1 2]) / 1000;
F = T.ConnectivityList;
n = size(F, 1);
if n > 6000
    [F, V] = reducepatch(F, V, max(6000, round(0.25 * n)));
end
end
