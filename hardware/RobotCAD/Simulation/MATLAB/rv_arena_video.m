function rv_arena_video(R, A, P, fn, opts)
%RV_ARENA_VIDEO  Render an arena run to an MP4 with the SolidWorks parts.
%   rv_arena_video(R, A, P, file) draws the course and the robot (chassis, wheels
%   turning, front caster carriage on its spring) from the simulated motion, with
%   a chase camera, a plan view inset and a status line. Works under
%   matlab -batch, so no Mechanics Explorer is needed.
%   Options: speedup (default 2, so a 100 s run gives a 50 s video), fps (30),
%   width/height (1280 x 720).

arguments
    R struct
    A struct
    P struct
    fn (1,1) string
    opts.speedup (1,1) double = 2
    opts.fps (1,1) double = 30
    opts.width (1,1) double = 1280
    opts.height (1,1) double = 720
    opts.frames double = []          % frame numbers to render (testing); default all
    opts.still string = ""           % also save this frame number's image here
end
V = make_rv_visuals(P);
f = figure(Visible="off", Position=[0 0 opts.width opts.height], Color=[0.93 0.95 0.98], Renderer="opengl");
ax = axes(f, Position=[0 0 1 1]); hold(ax, "on"); axis(ax, "equal", "off");
ax.Projection = "perspective"; ax.Clipping = "off";
ax.XLim = [-2 9]; ax.YLim = [-3 3]; ax.ZLim = [-0.1 1.5];

% course
for o = A.objects, draw_object(ax, o); end
for x = -1.5:0.5:8.5, plot3(ax, [x x], [-2.5 2.5], [1 1]*1e-3, Color=[0.6 0.6 0.58 0.5]); end
for y = -2.5:0.5:2.5, plot3(ax, [-1.5 8.5], [y y], [1 1]*1e-3, Color=[0.6 0.6 0.58 0.5]); end
plot3(ax, A.path(:,1), A.path(:,2), 2e-3 + 0*A.path(:,1), ":", Color=[0.1 0.4 0.1], LineWidth=1);
light(ax, Position=[2 -3 6], Style="infinite"); light(ax, Position=[-3 4 5], Style="infinite");
lighting(ax, "gouraud");

% robot: transforms nested like the multibody tree
hC = hgtransform(ax);
hW = struct("L", hgtransform(hC), "R", hgtransform(hC));
hK = hgtransform(hC);                                            % caster carriage
for v = V
    T = stlread(v.file);
    switch v.body
        case "chassis", par = hC;
        case "drawer", par = hC;
        case "carriage", par = hK;
        case "wheelL", par = hW.L;
        case "wheelR", par = hW.R;
    end
    patch(Parent=par, Faces=T.ConnectivityList, Vertices=T.Points, FaceColor=v.color, EdgeColor="none", ...
        FaceAlpha=max(v.opacity, 0.55), FaceLighting="gouraud", AmbientStrength=0.45, SpecularStrength=0.2);
end
trail = plot3(ax, nan, nan, nan, Color=[0.85 0.3 0.1], LineWidth=1.5);

% plan inset
ai = axes(f, Position=[0.71 0.03 0.28 0.3]); hold(ai, "on"); axis(ai, "equal"); box(ai, "on");
ai.XTick = []; ai.YTick = []; ai.Color = [1 1 1 0.85];
for o = A.objects, plan_object(ai, o); end
plot(ai, A.path(:,1), A.path(:,2), "k:");
ipath = plot(ai, nan, nan, Color=[0.85 0.3 0.1], LineWidth=1.5);
ibot = plot(ai, nan, nan, "-", Color=[0.1 0.2 0.8], LineWidth=1.5);
xlim(ai, [-1.7 8.7]); ylim(ai, [-2.7 2.7]);
hud = annotation(f, "textbox", [0.01 0.88 0.7 0.11], String="", FontSize=13, FontName="Consolas", ...
    EdgeColor="none", BackgroundColor=[1 1 1 0.75], FitBoxToText="off", VerticalAlignment="top");
title_box = annotation(f, "textbox", [0.01 0.01 0.68 0.05], String=sprintf( ...
    "RoamView arena run, %s spring (%.2f N/mm, %.0f N preload per spring)   %gx speed", ...
    R.spring, R.RV.k_spring/2e3, R.RV.F0_spring/2, opts.speedup), FontSize=11, EdgeColor="none"); %#ok<NASGU>

theta = cumtrapz(R.t, R.w);                                     % wheel angles
yaw_cam = R.yaw(1);
vw = VideoWriter(fn, "MPEG-4"); vw.FrameRate = opts.fps; vw.Quality = 90; open(vw);
cleanup = onCleanup(@() close(vw)); %#ok<NASGU>
tq = 0:opts.speedup/opts.fps:R.t(end);
B = A.box;
nf = numel(tq);
todo = 1:nf; if ~isempty(opts.frames), todo = opts.frames(opts.frames <= nf); end
fprintf("rendering %d frames to %s\n", nf, fn);
for n = todo
    k = min(numel(R.t), max(1, round(tq(n) / (R.t(2) - R.t(1))) + 1));
    Rm = quat2rot(R.Qw(k,:));
    hC.Matrix = [Rm [R.xw(k); R.yw(k); R.zw(k)]; 0 0 0 1];
    for s = ["L" "R"]
        sg = 1 - 2*(s == "R"); j = 1 + (s == "R");
        hW.(s).Matrix = makehgtform("translate", [0 sg*P.track/2 P.r_wheel], "xrotate", -pi/2, "zrotate", theta(k,j));
    end
    hK.Matrix = makehgtform("translate", [0 0 R.q_spring(k)]);
    kk = 1:20:k;
    set(trail, XData=R.xw(kk), YData=R.yw(kk), ZData=R.zw(kk) + 0.003);
    set(ipath, XData=R.xw(kk), YData=R.yw(kk));
    c = cos(R.yaw(k)); s_ = sin(R.yaw(k));
    pts = [B(1) -B(3); B(2) -B(3); B(2) B(3); B(1) B(3); B(1) -B(3)] * [c s_; -s_ c] + [R.xw(k) R.yw(k)];
    set(ibot, XData=pts(:,1), YData=pts(:,2));

    % chase camera, heading smoothed so turns in place don't whip the view
    dy = mod(R.yaw(k) - yaw_cam + pi, 2*pi) - pi;
    yaw_cam = yaw_cam + 0.06 * dy;
    tgt = [R.xw(k) R.yw(k) R.zw(k) + 0.18];
    off = [-1.55*cos(yaw_cam) + 0.75*sin(yaw_cam), -1.55*sin(yaw_cam) - 0.75*cos(yaw_cam), 1.05];
    campos(ax, tgt + off); camtarget(ax, tgt); camup(ax, [0 0 1]); camva(ax, 40);

    near = "";
    dm = vecnorm(reshape([A.marks.xy], 2, []).' - [R.xw(k) R.yw(k)], 2, 2);
    [dmin, im] = min(dm);
    if dmin < 0.6, near = A.marks(im).name; end
    flag = "";
    if R.Fcoll(k) > 1, flag = sprintf("  COLLISION %.0f N", R.Fcoll(k)); end
    if R.Fscrape(k) > 1, flag = flag + sprintf("  SCRAPE %.0f N", R.Fscrape(k)); end
    lift = ["L wheel" "R wheel" "front caster" "rear caster"];
    off_floor = lift(R.Fn_s(k,:) < 0.5 & R.t(k) > 0.5);
    if ~isempty(off_floor), flag = flag + "  LIFTED: " + strjoin(off_floor, ", "); end
    hud.String = {sprintf("t %5.1f s   speed %4.2f m/s   pitch %+5.1f   roll %+5.1f deg", R.t(k), R.vx(k), ...
        rad2deg(R.pitch(k)), rad2deg(R.roll(k))), ...
        sprintf("loads L %4.1f  R %4.1f  front %4.1f  rear %4.1f N   spring %4.1f mm", R.Fn_s(k,:), 1000*R.q_spring(k)), ...
        "section: " + near + flag};
    img = print(f, "-RGBImage", "-r0");
    img = img(1:2*floor(end/2), 1:2*floor(end/2), :);
    writeVideo(vw, img);
    if opts.still ~= "" && n == todo(ceil(end/2)), imwrite(img, opts.still); end
    if mod(n, 300) == 0, fprintf("  frame %d of %d\n", n, nf); end
end
close(f);
fprintf("video written: %s\n", fn);
end

function Rm = quat2rot(q)
q = q / norm(q); w = q(1); x = q(2); y = q(3); z = q(4);
Rm = [1-2*(y^2+z^2), 2*(x*y-w*z), 2*(x*z+w*y);
      2*(x*y+w*z), 1-2*(x^2+z^2), 2*(y*z-w*x);
      2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x^2+y^2)];
end

function M = obj_tf(o)
switch o.axis
    case "+X", Rm = makehgtform("xrotate", deg2rad(o.angle));
    case "+Y", Rm = makehgtform("yrotate", deg2rad(o.angle));
    otherwise, Rm = makehgtform("zrotate", deg2rad(o.angle));
end
M = makehgtform("translate", o.pos) * Rm;
end

function draw_object(ax, o)
h = hgtransform(ax, Matrix=obj_tf(o));
if o.shape == "brick"
    d = o.dims / 2;
    [X, F] = deal([-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] .* d, ...
        [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8]);
    a = 1; if o.role == "obstacle" && startsWith(o.name, "wall"), a = 0.55; end
    patch(Parent=h, Faces=F, Vertices=X, FaceColor=o.color, EdgeColor=o.color*0.7, FaceAlpha=a, FaceLighting="gouraud");
else
    [x, y, z] = cylinder(o.dims(1), 32);
    z = (z - 0.5) * o.dims(2);
    surface(x, y, z, Parent=h, FaceColor=o.color, EdgeColor="none", FaceLighting="gouraud");
    th = linspace(0, 2*pi, 33);
    patch(XData=o.dims(1)*cos(th), YData=o.dims(1)*sin(th), ZData=o.dims(2)/2 + 0*th, Parent=h, FaceColor=o.color, EdgeColor="none");
end
end

function plan_object(ai, o)
if o.name == "floor", return; end
if o.shape == "cyl" && o.axis == "+Z"
    th = linspace(0, 2*pi, 30);
    fill(ai, o.pos(1) + o.dims(1)*cos(th), o.pos(2) + o.dims(1)*sin(th), o.color, EdgeColor="none");
else
    d = o.dims;
    if o.shape == "cyl", d = [2*o.dims(1) o.dims(2)]; end
    if o.axis == "+Y", d(1) = d(1) * cosd(o.angle); end
    rectangle(ai, Position=[o.pos(1) - d(1)/2, o.pos(2) - d(2)/2, d(1), d(2)], FaceColor=o.color, EdgeColor="none");
end
end
