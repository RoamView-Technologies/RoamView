function A = rv_arena_layout(P)
%RV_ARENA_LAYOUT  The test course RoamView drives in rv_arena.slx.
%   Two rooms joined by a 0.8 m doorway with a 15 mm threshold. Room A has a
%   5 degree ramp up to a 50 mm platform and back down, a 12 mm cable cover,
%   three table legs to slalom through, a sofa against the wall and a bin.
%   The robot follows A.path (pure pursuit, rv_pursuit.m) from A.start.
%
%   Each object has a role:
%     ground    the wheels and casters roll on it, and the underside of the
%               chassis is checked for scraping on it
%     obstacle  the chassis footprint (body length x full width over the tyres)
%               is checked for collisions with it
%   Frame: world x east, y north, z up, metres. Rotations are about one axis,
%   in degrees, applied before the translation.

arguments
    P struct = rv_params()
end
th = 5;                      % ramp angle, deg
h = 0.05;                    % platform height
run = h / tand(th);
xr = 0.0;                    % foot of the up ramp
yl = -1.7;                   % lane of the ramp and the cable cover
wl = 0.9;                    % lane width
plat = 0.8;                  % platform length
wall_h = 0.35; wt = 0.1;
door = [1.1 1.9];            % doorway, y range
xd = 5.5;                    % internal wall, west face

floorc = [0.78 0.78 0.74]; wallc = [0.86 0.84 0.8]; rampc = [0.45 0.55 0.7];
woodc = [0.55 0.38 0.22]; sofac = [0.35 0.45 0.35]; binc = [0.25 0.25 0.3]; cablec = [0.95 0.75 0.1];

O = struct("name", {}, "shape", {}, "dims", {}, "pos", {}, "axis", {}, "angle", {}, "color", {}, "role", {});
    function add(name, shape, dims, pos, ax, ang, col, role)
        O(end+1) = struct("name", name, "shape", shape, "dims", dims, "pos", pos, "axis", ax, ...
            "angle", ang, "color", col, "role", role);
    end

% ground
add("floor", "brick", [12 7 0.02], [3.5 0 -0.01], "+Z", 0, floorc, "ground");
t = 0.03;                                                    % ramp slab thickness
add("ramp_up", "brick", [run/cosd(th) + 0.1, wl, t], ...
    [xr + run/2 - 0.05*cosd(th) + t/2*sind(th), yl, h/2 - 0.05*sind(th) - t/2*cosd(th)], "+Y", -th, rampc, "ground");
add("platform", "brick", [plat wl h], [xr + run + plat/2, yl, h/2], "+Z", 0, rampc, "ground");
x2 = xr + run + plat;
add("ramp_down", "brick", [run/cosd(th) + 0.1, wl, t], ...
    [x2 + run/2 + 0.05*cosd(th) - t/2*sind(th), yl, h/2 - 0.05*sind(th) - t/2*cosd(th)], "+Y", th, rampc, "ground");
rb = 0.025; hb = 0.012;                                      % cable cover: 12 mm high round hump
add("cable_cover", "cyl", [rb wl], [2.8 yl hb - rb], "+X", 90, cablec, "ground");
add("threshold", "brick", [0.04 diff(door) 0.015 + 0.002], [xd + wt/2, mean(door), 0.0075 - 0.001], "+Z", 0, woodc, "ground");

% walls
xa = -1.5; xb = 8.5; ya = -2.5; yb = 2.5;
add("wall_S", "brick", [xb - xa + 2*wt, wt, wall_h], [(xa + xb)/2, ya - wt/2, wall_h/2], "+Z", 0, wallc, "obstacle");
add("wall_N", "brick", [xb - xa + 2*wt, wt, wall_h], [(xa + xb)/2, yb + wt/2, wall_h/2], "+Z", 0, wallc, "obstacle");
add("wall_W", "brick", [wt, yb - ya, wall_h], [xa - wt/2, 0, wall_h/2], "+Z", 0, wallc, "obstacle");
add("wall_E", "brick", [wt, yb - ya, wall_h], [xb + wt/2, 0, wall_h/2], "+Z", 0, wallc, "obstacle");
add("wall_mid_S", "brick", [wt, door(1) - ya, wall_h], [xd + wt/2, (ya + door(1))/2, wall_h/2], "+Z", 0, wallc, "obstacle");
add("wall_mid_N", "brick", [wt, yb - door(2), wall_h], [xd + wt/2, (yb + door(2))/2, wall_h/2], "+Z", 0, wallc, "obstacle");
% furniture
for k = 1:3
    add("table_leg_" + k, "cyl", [0.04 0.72], [3.4 - 1.2*(k-1), 0, 0.36], "+Z", 0, woodc, "obstacle");
end
add("sofa", "brick", [1.6 0.5 0.45], [2.2 2.2 0.225], "+Z", 0, sofac, "obstacle");
add("bin", "cyl", [0.15 0.4], [4.3 0.85 0.2], "+Z", 0, binc, "obstacle");
add("cabinet", "brick", [0.6 0.4 0.8], [7.9 2.2 0.4], "+Z", 0, woodc, "obstacle");
A.objects = O;

% route: ramp lane east, turn north, slalom west through the table legs,
% round to the north lane, past the sofa and the bin, through the doorway, park in room B
A.start = [-1.0 yl 0];
wp = [-1.0 yl; 3.9 yl; 4.5 -1.3; 4.6 -0.6; 4.2 -0.1;
      3.4 -0.6; 2.8 0; 2.2 0.6; 1.6 0; 1.0 -0.6; 0.3 -0.25;
      -0.5 0.3; -0.6 1.0; -0.1 1.5; 4.9 1.5; 6.6 1.5; 7.4 1.1; 7.6 0.3; 7.6 -0.5];
A.waypoints = wp;
A.path = densify(wp, 0.02);
% where things happen along the route (for the report)
A.marks = struct("name", {"ramp up", "platform", "ramp down", "cable cover", "slalom", "sofa", "bin", "doorway threshold", "parked"}, ...
    "xy", {[xr + run/2 yl], [xr + run + plat/2 yl], [x2 + run/2 yl], [2.8 yl], [2.2 0], [2.2 1.5], [4.3 1.5], [xd + wt/2 1.5], wp(end,:)});
A.box = cad_box(P);                             % chassis envelope for the collision and scrape checks
A.speed = 0.4;                                  % cruise, m/s
A.lookahead = 0.3;                              % pure pursuit, m
A.tstop = 110;
end

function p = densify(wp, ds)
p = wp(1,:);
for k = 2:size(wp, 1)
    n = max(1, ceil(norm(wp(k,:) - wp(k-1,:)) / ds));
    s = (1:n)' / n;
    p = [p; wp(k-1,:) + s .* (wp(k,:) - wp(k-1,:))]; %#ok<AGROW>
end
end

function b = cad_box(P)
% Plan extents of the whole robot and the lowest point of the chassis (casters and
% wheels excluded) from the part STLs; [xmin xmax ymax zmin ztop] in metres, ROS frame.
% Falls back to the globals if the STLs are missing.
src = fullfile(P.dirs.sim, "FOV", "parts");
f = dir(fullfile(src, "*.STL"));
b = [-P.body(1)/2, P.body(1)/2, P.track/2 + 0.026, P.G.FLOOR_Z/1000, P.G.DECK_Z/1000];
if isempty(f), return; end
lo = inf; xr = [inf -inf]; ym = 0;
for k = 1:numel(f)
    T = stlread(fullfile(src, f(k).name));
    V = T.Points(:, [3 1 2]) / 1000;
    xr = [min(xr(1), min(V(:,1))), max(xr(2), max(V(:,1)))];
    ym = max(ym, max(abs(V(:,2))));
    if isempty(regexp(f(k).name, "RV-50[234]_|RV-30[89]_|RV-310_", "once")), lo = min(lo, min(V(:,3))); end
end
b = [xr, ym, lo, P.G.DECK_Z/1000];
end
