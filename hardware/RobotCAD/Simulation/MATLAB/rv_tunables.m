function RV = rv_tunables(P, opts)
%RV_TUNABLES  Per-run settings read by rv_dynamics.slx (the RV variable).
%   RV = rv_tunables(P) gives the nominal robot on a flat floor.
%   Name-value options change one run without touching the model:
%     ramp_deg      floor slope, uphill along +x (default 0)
%     x_thr         x of the near edge of a door threshold, m (default far away)
%     h_thr, l_thr  threshold height and length along x, m (default 15 mm, 40 mm)
%     direction     +1 forward, -1 reverse: sets which way the caster wheels trail
%     drawer_out    battery drawer pulled out by this much, m (moves the COM)
%     k_spring      front caster spring rate, both springs, N/m
%     F0_spring     front caster preload, both springs, N
%     pwm_cap       duty limit (default P.driver.pwm_cap)
%     Vbatt         pack voltage (default full charge, 16.8 V)
%     tip_worst     true: both caster trails turned inward (smallest support polygon)
%     x0            starting x of the robot origin, m
%     caster_d      caster wheel diameter, m (default CASTER_D; 0.075 = 3-inch option)
%     travel        front carriage compression travel, m (default 10 mm)
%     droop         front carriage extension available below its rest point, m (default 0:
%                   the carriage rests on its lower stop)

arguments
    P struct
    opts.ramp_deg (1,1) double = 0
    opts.x_thr (1,1) double = 1e3
    opts.h_thr (1,1) double = 0.015
    opts.l_thr (1,1) double = 0.040
    opts.direction (1,1) double = 1
    opts.drawer_out (1,1) double = 0
    opts.k_spring (1,1) double = P.spring.k
    opts.F0_spring (1,1) double = P.spring.F0
    opts.pwm_cap (1,1) double = P.driver.pwm_cap
    opts.Vbatt (1,1) double = P.batt.V_full
    opts.tip_worst (1,1) logical = false
    opts.x0 (1,1) double = 0
    opts.caster_d (1,1) double = 2 * P.r_caster
    opts.travel (1,1) double = P.spring_travel
    opts.droop (1,1) double = 0
    opts.scene (1,1) string {mustBeMember(opts.scene, ["test" "arena"])} = "test"
    opts.start (1,3) double = [0 0 0]
end

th = deg2rad(opts.ramp_deg);
RV.gvec = P.g * [-sin(th) 0 -cos(th)];

% casters: wheel trails behind the swivel in the direction of travel
d = sign(opts.direction);
if opts.tip_worst
    RV.xF = P.caster_x - P.caster_trail;
    RV.xR = -P.caster_x + P.caster_trail;
else
    RV.xF = P.caster_x - d * P.caster_trail;
    RV.xR = -P.caster_x - d * P.caster_trail;
end
RV.r_caster = opts.caster_d / 2;
RV.zF = RV.r_caster - P.spring.preset;

% chassis (drawer pulled out moves its mass rearward; drawer runs along -x)
RV.m_chassis = P.m_chassis;
RV.com_chassis = P.com_chassis + [-opts.drawer_out * P.m_drawer / P.m_chassis 0 0];
RV.drawer_out = opts.drawer_out;                      % slides the drawer mesh in the CAD look
J = P.J_chassis;
RV.J_moments = [J(1,1) J(2,2) J(3,3)];
RV.J_products = [J(2,3) J(3,1) J(1,2)];

% wheels and casters
Js = 0.6 * P.m_wheel * P.r_wheel^2 + P.motor.J_out;   % tyre-dominated wheel + reflected rotor
Jt = P.m_wheel * (3 * 0.6 * P.r_wheel^2 + P.w_wheel^2) / 12;
RV.m_wheel = P.m_wheel;
RV.J_wheel = [Jt Jt Js];
RV.m_carriage = P.m_carriage + P.m_caster_fork;
RV.m_cwheel = P.m_caster_wheel;
RV.m_rear = P.m_caster;

% spring
RV.k_spring = opts.k_spring;
RV.F0_spring = opts.F0_spring;
RV.c_spring = P.spring.c;
RV.travel = opts.travel;
RV.droop = opts.droop;

% contact
RV.k_contact = 2e5;
RV.c_contact = 1.5e3;
RV.mu_s = P.mu + 0.05;
RV.mu_d = P.mu;

% ground features
RV.x_thr = opts.x_thr + opts.l_thr / 2;
RV.h_thr = opts.h_thr;
RV.l_thr = opts.l_thr;

% drive
RV.T_roll = P.Crr * P.m_total * P.g / 2 * P.r_wheel;   % per wheel; same value is baked into the motor Fcn
RV.Kp = P.ctrl.Kp;
RV.Ki = P.ctrl.Ki;
RV.pwm_cap = opts.pwm_cap;
RV.Vbatt = opts.Vbatt;

% start pose
RV.x0 = opts.x0;
RV.model = "rv_dynamics"; if opts.scene == "arena", RV.model = "rv_arena"; end
RV.scene = opts.scene;
RV.start = opts.start;                    % arena start pose [x y yaw]
RV.z0 = 0.001;
end
