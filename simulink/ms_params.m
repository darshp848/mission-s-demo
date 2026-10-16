function [P, info] = ms_params()
%MS_PARAMS  Parameters for the Simulink Mission S stunt model.
%   Thruster geometry and channel data come from ../thruster_layout.json.
%   P is what the flight computer believes (nominal). The plant reads PP,
%   which run_mission_s sets equal to P and ms_robustness perturbs.

P.g = 9.81;
% ---- Base car (918 Spyder proxy; Mission S figures unpublished)
P.m_car = 1675; P.L = 4.61; P.W = 2.00; P.H = 1.16;
P.h_ref = 0.55; P.wheel_r = 0.335; P.CdA = 0.75; P.rho = 1.225; P.mu = 1.15;
% ---- Powertrain
P.eng_rpm = [1000 3000 5000 6700 8700 9150];
P.eng_Nm  = [280 420 500 540 491 450];
P.redline = 9150; P.launch_rpm = 4500;
P.gears = [3.91 2.35 1.69 1.28 1.04 0.86 0.70]; P.final_drive = 3.33; P.eta = 0.90;
P.em_front_kW = 95; P.em_front_Fmax = 6000; P.em_rear_kW = 115; P.em_rear_Fmax = 4000;
% ---- Run-up (no ramp)
P.t_hold = 1.0; P.v_cruise = 12.0; P.x_liftoff = 22.0;
% ---- Lineup: width along path, length across, height [m]
info.lineup_names = {'Model 3', 'Cybercab', 'Semi', 'Cybertruck', 'Model Y'};
P.lineup = [1.85 4.72 1.44; 1.95 4.00 1.45; 2.60 7.00 4.10; 2.20 5.68 1.90; 1.92 4.75 1.62];
P.lineup_gap = 1.2; P.lineup_start = 18.0;
x0 = P.x_liftoff + P.lineup_start;
P.lineup_x = zeros(5, 2);
for i = 1:5
    P.lineup_x(i, :) = [x0, x0 + P.lineup(i, 1)];
    x0 = x0 + P.lineup(i, 1) + P.lineup_gap;
end
% ---- Engines
P.SD_T = 71000; P.SD_tmin = 0.20; P.SD_Isp = 235; P.SD_tau = 0.08;   % SuperDraco
P.DR_T = 400;   P.DR_Isp = 220;   P.DR_tau = 0.03;                    % Draco
P.n_roll = 16; P.n_pitch = 8;
layout = jsondecode(fileread(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'thruster_layout.json')));
channels = layout.channels;
P.n_thr = numel(channels);
P.thr_pos = zeros(P.n_thr, 3); P.thr_dir = zeros(P.n_thr, 3);
P.is_SD = zeros(P.n_thr, 1); P.Tmax = zeros(P.n_thr, 1);
P.Isp = zeros(P.n_thr, 1); P.tau = zeros(P.n_thr, 1);
info.groups = cell(1, P.n_thr);
for k = 1:P.n_thr
    c = channels(k);
    info.groups{k} = c.name;
    P.thr_pos(k,:) = c.position_m(:).';
    P.thr_dir(k,:) = c.force_direction(:).';
    P.is_SD(k) = double(strcmp(c.kind, 'superdraco'));
    P.Tmax(k) = c.max_thrust_N; P.Isp(k) = c.Isp_s; P.tau(k) = c.lag_s;
end
assert(P.n_thr == 28 && sum(P.is_SD) == 2, 'Unexpected canonical channel count.');
assert(max(abs(vecnorm(P.thr_dir, 2, 2)-1)) < 1e-10, 'Thruster directions must be unit vectors.');
% ---- Propellant, tanks, pressurant
P.m_dry_prop = 220; P.dry_pos = [-0.3 0 -0.15];
P.OF = 1.65; P.m_prop0 = 140; P.rho_f = 880; P.rho_o = 1440; P.ullage0 = 0.10;
P.fuel_pos = [0.75 0 -0.10]; P.ox_pos = [-0.75 0 -0.05];
P.copv_V = 0.020; P.copv_P0 = 300e5; P.P_reg = 25e5; P.R_he = 2077; P.T_gas = 293;
% ---- Flight plan
P.a_boost = 9.0; P.t_boost = 1.45; P.t_roll_delay = 0.05; P.T_roll = 1.55;
P.roll_dir = -1;            % roll sense; -1 = right side up first, so the car drifts left
P.roll_sd_throttle = 0.21;  % SuperDraco throttle (each) while they drive the sidestep mid-roll
P.roll_sd_window = 0.3;     % rad of roll before the SuperDracos may light
P.omega_pitch = 5.0; P.pitch_gap = 0.15;
P.a_burn = 6.0; P.v_touch = 1.0; P.k_vz = 10.0; P.burn_margin = 0.4;
P.a_brake = 10.0; P.t_end_hold = 1.5;
% ---- Simulink-only additions (not in the Python sim)
P.y_side = 2.0;          % m sidestep to the left during the barrel roll (one body width, like the Roadster)
P.vy_side_max = 3.0;     % m/s max sidestep drift speed
P.k_vy = 3;              % 1/s drift-speed gain while rolling
P.k_y_pos = 2;           % 1/s sidestep position gain while rolling
P.ay_sd_on = 1.0;        % m/s^2 lateral demand above which the SuperDracos light mid-roll
P.k_y1 = 4; P.k_y2 = 4;  % lateral position/velocity gains in boost / landing burn
P.ay_max = 6;            % m/s^2 lateral accel command limit
P.tilt_max = 0.2;        % rad max thrust tilt for lateral control on the SuperDracos
P.tilt_fade_h = 2.5;     % m tilt limit fades to zero below this height
P.Fpod_max = 6000;       % N max net pod force for the sidestep
P.roll_auth_frac = 0.75; % roll profile may use at most this share of pod roll authority
P.settle_timeout = 0.5;  % s max wait for the roll to settle before the backflip
P.h_flip_min = 9.0;      % m lowest body height at which the backflip may start (flip + catch use ~7 m)
% ---- Rates
P.Ts = 0.005;     % flight computer, 200 Hz
P.dt = 0.001;     % plant integration step (ode4)
P.fps = 24;
% ---- Attitude LQR, state [int(e); e; e_dot; a_hat - a_ref], input = angular accel cmd
P.lqr_Q = [2e4 1e4 60 0]; P.lqr_R = 1;
P.K_roll     = ms_lqr_axis(P.DR_tau, P.lqr_Q, P.lqr_R);   % Draco pods
P.K_pitch    = ms_lqr_axis(P.DR_tau, P.lqr_Q, P.lqr_R);   % Draco pods
P.K_pitch_SD = ms_lqr_axis(P.SD_tau, P.lqr_Q, P.lqr_R);   % SuperDraco differential
P.K_yaw = ms_lqr_axis(P.DR_tau, P.lqr_Q, P.lqr_R);
P.int_lim = 0.5;
P.k_x = 1.5; P.k_vx = 2;
P.alloc_weights = [1/5000; 1/5000; 1/10000; 1/200; 1/400; 1/200];
P.q_norm_gain = 2;
% ---- Ground contact (suspension + tyres)
P.k_gnd = 3e5; P.zeta_gnd = 0.6; P.w_att_gnd = 12; P.c_lat = 20;
% ---- Plant-only truth offsets (zero/one nominal; ms_robustness perturbs them)
P.cg_car_off = [0 0 0]; P.I_scale = [1 1 1]; P.thr_scale = ones(P.n_thr, 1);
% ---- Derived
P = ms_derive(P);
P.mem0 = zeros(22, 1);
P.mem0(11:13) = [-1; -1; P.T_roll]; P.mem0(17) = 1;
P.trac_v = 0:0.1:45;
P.trac_F = arrayfun(@(v) ms_tractive(P, v), P.trac_v);

info.mode_names = {'hold','launch','boost','coast','roll','roll_settle','pitch_kick', ...
    'pitch_coast','pitch_catch','descent','landing_burn','settle','rollout','rollout'};
end
