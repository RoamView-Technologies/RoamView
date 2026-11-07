function y = rv_pursuit(u, path, opts)
%RV_PURSUIT  Pure pursuit path follower for rv_arena.slx (runs at 50 Hz).
%   rv_pursuit("init", path, struct(speed=0.4, lookahead=0.3, t0=0.5)) loads a
%   route ([x y] rows, densely spaced) before a run.
%   y = rv_pursuit(u) is called by the Interpreted MATLAB Function block with
%   u = [t x y qw qx qy qz] (chassis pose in the world) and returns [v; yaw_rate].
%   rv_pursuit("state") returns the follower's state (progress along the route).
%   It slows for sharp heading errors, turns in place above 70 degrees, and
%   stops at the end of the route.

persistent S
if isstring(u) || ischar(u)
    switch string(u)
        case "init"
            S = struct("path", path, "idx", 1, "L", opts.lookahead, "v", opts.speed, ...
                "t0", opts.t0, "done", false, "s", [0; cumsum(vecnorm(diff(path), 2, 2))]);
            y = [];
        case "state"
            y = S;
    end
    return
end
y = [0; 0];
if isempty(S) || u(1) < S.t0 || S.done, return; end
p = u(2:3).';
q = u(4:7);
yaw = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));

n = size(S.path, 1);
win = S.idx:min(n, S.idx + 60);                       % search ahead only: the route crosses itself
[~, k] = min(sum((S.path(win,:) - p).^2, 2));
S.idx = win(k);
j = S.idx;
while j < n && norm(S.path(j,:) - p) < S.L, j = j + 1; end
d = S.path(j,:) - p;
dist_end = (S.s(end) - S.s(S.idx)) + norm(S.path(S.idx,:) - p);
if S.idx >= n - 1 && norm(S.path(n,:) - p) < 0.04
    S.done = true;
    return
end
alpha = mod(atan2(d(2), d(1)) - yaw + pi, 2*pi) - pi;
if abs(alpha) > deg2rad(70)
    v = 0;
    w = sign(alpha) * 1.2;                               % turn in place
else
    v = S.v * max(0.35, 1 - abs(alpha) / deg2rad(70));
    v = min(v, 0.6 * dist_end + 0.04);                  % ease in at the end
    w = 2 * v * sin(alpha) / max(norm(d), 0.05);
    w = max(-2, min(2, w));
end
y = [v; w];
end
