clc; clear all; close all;

% Setup
n = input('Enter number of harmonics (n): ');
a0 = 0;
c0 = 0;
direction = 1;
[a, b, c, d] = generate_curve(n);

% Generate Curves
[xt, yt] = get_efd_curve(a0, c0, a, b, c, d, n);
[f_ip] = efd_to_ip(a0, c0, a, b, c, d, n);

% Plot Setup
figure('Name', 'Nonholonomic Formation Control', 'Color', 'w');
plot_efd(xt, yt);
plot_ip(f_ip);
title('Nonholonomic Robots Tracking Virtual Ghosts');

%% 2. Initialization
N = 5; % Number of robots

% --- GHOST STATES (Point Particles) ---
x = zeros(1, N); 
y = zeros(1, N);
theta = zeros(1, N);
% Initial positions (Ghosts)
x(1) = 2.0;   y(1) = 2.0;   theta(1) = pi;
x(2) = 2.5;   y(2) = 1.5;   theta(2) = pi;
x(3) = 1.0;   y(3) = 1.5;   theta(3) = pi;
x(4) = 1.5;   y(4) = 1.0;   theta(4) = pi;
x(5) = 2.0;   y(5) = 1.0;   theta(5) = pi;

% --- REAL ROBOT STATES (Nonholonomic) ---
rx = x;          % Start real robots at same spot as ghosts
ry = y;
rtheta = theta; % Initial orientation (facing East)

% --- CONTROL PARAMETERS ---
lambda = 3;             % Formation gain
d_desired = 0.25;        % Coordination distance
k_shape = 2;            % Spring constant (Shape phase)
k_keep = 1;             % Spring constant (Keep phase)

% Nonholonomic Tracking Gains (Tunable)
k1 = 2.0; % Gain for e1 (Longitudinal)
k2 = 5.0; % Gain for e3 (Orientation)
k3 = 20; % Gain for e2 (Lateral Gain)

% Simulation Vars
dt = 0.05;
v_max_ghost = 0.1;      % Max speed of the ghost
omega_ghost = v_max_ghost/2;     % Angular velocity of parameter t

% Real Robot Actuator Limits (Prevents infinite speed)
v_max_robot = 0.1;   % Max linear speed (m/s)
w_max_robot = 1;   % Max turning speed (rad/s)

% Memory
status = zeros(1, N);   
t = zeros(1, N);        
traj_synced = zeros(1, N); 
t_ref = linspace(0, 2*pi, 200); 
prev_theta_r = rtheta;
prev_vx = zeros(1, N);
prev_vy = zeros(1, N);

plot_initial_setup(rx, ry, rtheta, N);
%% 3. Main Loop
while true
    for i = 1:N
        % --- A. GHOST SIMULATION (POINT PARTICLE) ---
        
        % 1. Check Status 
        [status(i), k] = check_curve_status(f_ip, x(i), y(i), k_shape, k_keep);
        
        % 2. Formation Control (Ghost)
        if status(i) == 0
            % PHASE 1: APPROACH
            [u_form, v_form] = formation_ip(f_ip, x(i), y(i), lambda);
            traj_synced(i) = 0; 
        else
            % PHASE 2: MAINTAIN
            if traj_synced(i) == 0
                t(i) = get_nearest_t(xt, yt, x(i), y(i), t_ref);
                traj_synced(i) = 1; 
            end
            
            % Parametric Trajectory
            [xt_star, yt_star, xt_sd, yt_sd] = trajectory(a0, c0, a, b, c, d, n, t(i), omega_ghost);
            [u_form, v_form] = formation_efd(xt_star, yt_star, xt_sd, yt_sd, x(i), y(i), lambda);
            
            % Update Time
            t(i) = t(i) + dt * omega_ghost * direction; 
            t(i) = mod(t(i), 2*pi);
        end
        
        % 3. Coordination Control (Ghost)
        [xp, yp, xq, yq, dip, diq] = nearest_two(x, y, i);
        [u_coord, v_coord] = coordination(xp, yp, xq, yq, dip, diq, d_desired, x(i), y(i), k);
        
        % 4. Total Ghost Velocity
        gx_dot = u_form + u_coord;
        gy_dot = v_form + v_coord;
        
        % 5. Velocity Saturation (Ghost)
        current_speed = sqrt(gx_dot^2 + gy_dot^2);
        if current_speed > v_max_ghost
            scale = v_max_ghost / current_speed;
            gx_dot = gx_dot * scale;
            gy_dot = gy_dot * scale;
        end
        
        % --- B. NONHOLONOMIC CONTROL (REAL ROBOT) ---
        
        % 1. Calculate Ghost Acceleration
        gx_ddot = (gx_dot - prev_vx(i)) / dt;
        gy_ddot = (gy_dot - prev_vy(i)) / dt;
        prev_vx(i) = gx_dot; 
        prev_vy(i) = gy_dot;
        
        % 2. Reference Dynamics (FIXED with Memory)
        % We pass prev_theta_r(i) to keep orientation stable
        [u1_r, u2_r, theta_r] = get_reference_dynamics(gx_dot, gy_dot, gx_ddot, gy_ddot, prev_theta_r(i));
        prev_theta_r(i) = theta_r; % Update memory for next loop
        
        % 3. Calculate Tracking Errors
        [e1, e2, e3] = get_tracking_error(x(i), y(i), theta_r, rx(i), ry(i), rtheta(i));
        
        % 4. Calculate Control Inputs
        [u1, u2] = calculate_nonholonomic_control(e1, e2, e3, u1_r, u2_r, k1, k2, k3);
        
        % 5. ACTUATOR SATURATION (CRITICAL FIX)
        % This forces the robot to obey physical limits
        if abs(u1) > v_max_robot
            u1 = sign(u1) * v_max_robot;
        end
        
        if abs(u2) > w_max_robot
            u2 = sign(u2) * w_max_robot;
        end

        % --- C. INTEGRATION ---
        
        % 1. Update Ghost Position
        x(i) = x(i) + gx_dot * dt;
        y(i) = y(i) + gy_dot * dt;
        
        % 2. Update Real Robot Position (Nonholonomic Dynamics)
        [rx_dot, ry_dot, rtheta_dot] = nonholonomic_dynamics(u1, u2, rtheta(i));
        
        rx(i) = rx(i) + rx_dot * dt;
        ry(i) = ry(i) + ry_dot * dt;
        rtheta(i) = rtheta(i) + rtheta_dot * dt;
        
        % --- D. PLOTTING ---
        plot_simulation(x(i), y(i), rx(i), ry(i), rtheta(i), i);
    end
    pause(dt);
end

%% FUNCTION DEFINITIONS
% Function 00: Random Curve Generation
function [a, b, c, d] = generate_curve(n)
    % We generate a shape centered at (0,0) first to get the shape coefficients
    N_points = 1000;
    t = linspace(0, 2*pi, N_points);
    
    % Start with a base circle
    r = 1.5 * ones(size(t)); 
    
    % Add random "wobbles" to make it a unique shape
    % We use low frequencies (2, 3, 4) to keep it smooth and closed
    for k_rand = 2:10
        noise_mag = 0.3 * rand(); % Random magnitude
        phase = 2*pi * rand();    % Random phase
        r = r + noise_mag * cos(k_rand * t + phase);
    end
    
    % Convert to Cartesian (Centered at 0,0)
    x_raw = r .* cos(t);
    y_raw = r .* sin(t);
    
    % Force row vectors for calculation
    x_raw = reshape(x_raw, 1, []);
    y_raw = reshape(y_raw, 1, []);
    
    t_idx = 1:N_points;
    theta = t_idx * (2*pi / N_points);
    
    a = zeros(1, n); 
    b = zeros(1, n);
    c = zeros(1, n); 
    d = zeros(1, n);
    
    for k = 1:n
        % Standard EFD formulas
        a(k) = (2 / N_points) * sum(x_raw .* cos(k * theta));
        b(k) = (2 / N_points) * sum(x_raw .* sin(k * theta));
        c(k) = (2 / N_points) * sum(y_raw .* cos(k * theta));
        d(k) = (2 / N_points) * sum(y_raw .* sin(k * theta));
    end

end
% Function 0: Plot Initial Pose & Setup Static Legend
function plot_initial_setup(x, y, theta, N)
    hold on;
    colors = ['r', 'g', 'b', 'c', 'm'];
    
    % --- 1. Plot Initial Positions (Static Squares) ---
    for i = 1:N
        c = colors(mod(i-1, length(colors))+1);
        
        % Plot Initial Position (Square Marker)
        plot(x(i), y(i), 's', 'MarkerSize', 8, ...
             'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'LineWidth', 1);
         
        % Plot Initial Orientation (Dotted Line)
        len = 0.1;
        plot([x(i), x(i) + len*cos(theta(i))], ...
             [y(i), y(i) + len*sin(theta(i))], ...
             'k:', 'LineWidth', 1.5);
    end
    
    % --- 2. Setup Legend (Using Invisible Dummies) ---
    % We plot NaNs (Not a Number) to create legend entries without cluttering the graph
    h_ghost_path = plot(nan, nan, 'k--', 'LineWidth', 1); 
    h_robot_path = plot(nan, nan, 'k-', 'LineWidth', 2);
    h_initial    = plot(nan, nan, 'ks', 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
    h_current    = plot(nan, nan, 'ko', 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k');
    
    legend([h_ghost_path, h_robot_path, h_initial, h_current], ...
           {'Ghost Path', 'Robot Path', 'Initial Pose', 'Current Robot'}, ...
           'Location', 'bestoutside', 'AutoUpdate', 'off'); 
           % AutoUpdate off prevents the loop from adding new entries
end
% Function 1: Generate Parametric Points (EFD)
function [xt, yt] = get_efd_curve(a0, c0, a, b, c, d, n)
    t = linspace(0, 2*pi, 200);
    xt = a0 * ones(size(t));
    yt = c0 * ones(size(t));
    for k = 1:n
        xt = xt + a(k)*cos(k*t) + b(k)*sin(k*t);
        yt = yt + c(k)*cos(k*t) + d(k)*sin(k*t);
    end
end
% Function 2: Convert to Implicit Polynomial
function [f_ip] = efd_to_ip(a0, c0, a, b, c, d, n)
    A = (a - 1i*b)/2; B = (a + 1i*b)/2; C = (c - 1i*d)/2; D = (c + 1i*d)/2;
    g = [fliplr(B), a0, A]; h = [fliplr(D), c0, C];
    deg = 2 * n; monos = [];
    for k = 0:deg
        for q = 0:k, monos = [monos; k-q, q]; end
    end
    width = 2*(n*deg) + 1; M = zeros(size(monos,1), width); center_idx = n*deg + 1;
    for i = 1:size(monos,1)
        p = monos(i,1); q = monos(i,2);
        res = 1;
        for k=1:p, res = conv(res, g); end
        for k=1:q, res = conv(res, h); end
        l = length(res); M(i, center_idx-floor(l/2) : center_idx+floor(l/2)) = res;
    end
    null_vec = null([real(M), imag(M)]'); ip_coeffs = null_vec(:, end);
    f_ip = @(x,y) 0;
    for i = 1:length(ip_coeffs)
        p = monos(i,1); q = monos(i,2); coeff = ip_coeffs(i);
        f_ip = @(x,y) f_ip(x,y) + coeff * (x.^p) .* (y.^q);
    end
end
% Function 3: Plot EFD
function plot_efd(xt, yt)
    hold on; axis equal; grid on; xlabel('X'); ylabel('Y');
    plot(xt, yt, 'b-', 'LineWidth', 2);
end
% Function 4: Plot IP
function plot_ip(f_ip)
    ax = axis; margin = 0.5; bounds = [ax(1)-margin, ax(2)+margin, ax(3)-margin, ax(4)+margin];
    fp = fimplicit(f_ip, bounds); fp.Color = 'r'; fp.LineStyle = ':'; fp.LineWidth = 3; fp.MeshDensity = 150; 
end
%% Holonomic Robot Control
% Function 5: Test Robot Status
function [status, k] = check_curve_status(f_ip, xi, yi, k_shapeFormation, k_keepFormation)
    tolerance = 0.1; val = f_ip(xi, yi);
    if abs(val) <= tolerance, status = 1; k = k_keepFormation; else, status = 0; k = k_shapeFormation; end
end
% Function 6: Calculate Formation Velocity IP
function [u_form, v_form] = formation_ip(f_ip, xi, yi, lambda)
    h = 1e-6; F_val = f_ip(xi, yi);
    Fx = (f_ip(xi + h, yi) - F_val) / h; Fy = (f_ip(xi, yi + h) - F_val) / h;
    grad_sq_norm = Fx^2 + Fy^2; scalar_term = -lambda * (1 / (grad_sq_norm + eps)) * F_val;
    u_form = scalar_term * Fx; v_form = scalar_term * Fy;
end
% Function 7: Get Trajectory 
function [xt_star, yt_star, xt_star_dot, yt_star_dot] = trajectory(a0, c0, a, b, c, d, n, ti, w)
    % Initialize Position
    xt_star = a0;
    yt_star = c0;
    
    % Initialize Velocity
    xt_star_dot = 0;
    yt_star_dot = 0;
    
    for k = 1:n
        % Position Sums (theta = k * ti)
        xt_star = xt_star + a(k)*cos(k*ti) + b(k)*sin(k*ti);
        yt_star = yt_star + c(k)*cos(k*ti) + d(k)*sin(k*ti);
        
        % Velocity Sums (Chain Rule: d/dt cos(w*t) = -w * sin(w*t))
        % We multiply the velocity by 'w' (omega) to slow it down mathematically
        xt_star_dot = xt_star_dot + k*w * (-a(k)*sin(k*ti) + b(k)*cos(k*ti));
        yt_star_dot = yt_star_dot + k*w * (-c(k)*sin(k*ti) + d(k)*cos(k*ti));
    end
end
% Function 8: Calculate Formation Velocity EFD
function [u_form, v_form] = formation_efd(xt_star, yt_star, xt_star_dot, yt_star_dot, xi, yi, lambda)
    ex = xt_star - xi; ey = yt_star - yi;
    u_form = lambda * ex + xt_star_dot; v_form = lambda * ey + yt_star_dot;
end
% Function 9: Extract Nearest 2 robots
function [xp, yp, xq, yq, dip, diq] = nearest_two(x, y, i)
    N = length(x); dists = zeros(N, 1); current_x = x(i); current_y = y(i);
    for k = 1:N
        if k == i, dists(k) = inf; else, dists(k) = sqrt((x(k) - current_x)^2 + (y(k) - current_y)^2); end
    end
    [sorted_dists, sorted_indices] = sort(dists);
    xp = 0; yp = 0; dip = 0; xq = 0; yq = 0; diq = 0;
    if N >= 2 && sorted_dists(1) < inf, idx_p = sorted_indices(1); xp = x(idx_p); yp = y(idx_p); dip = sorted_dists(1); end
    if N >= 3 && sorted_dists(2) < inf, idx_q = sorted_indices(2); xq = x(idx_q); yq = y(idx_q); diq = sorted_dists(2); end
end
% Function 10: Calculate Coordination Velocity (UPDATED with Division)
function [u_coord, v_coord] = coordination(xp, yp, xq, yq, dip, diq, d_desired, xi, yi, k)
    u_coord_p = 0; v_coord_p = 0;
    u_coord_q = 0; v_coord_q = 0;
    
    % Interaction with P 
    if dip > 0
        % Added division by dip to normalize the vector
        u_coord_p = k * (d_desired - dip) * (xi - xp) / dip;
        v_coord_p = k * (d_desired - dip) * (yi - yp) / dip;
    end
    
    % Interaction with Q
    if diq > 0
        % Added division by diq
        u_coord_q = k * (d_desired - diq) * (xi - xq) / diq;
        v_coord_q = k * (d_desired - diq) * (yi - yq) / diq;
    end
    
    u_coord = u_coord_p + u_coord_q;
    v_coord = v_coord_p + v_coord_q;
end
% Function 11: Find nearest parametric time t
function t_val = get_nearest_t(curve_x, curve_y, robot_x, robot_y, t_vector)
    dx = curve_x - robot_x; dy = curve_y - robot_y; dists = dx.^2 + dy.^2;
    [~, min_idx] = min(dists); t_val = t_vector(min_idx);
end
% Function 12: Total Control and Integration
function [xi, yi] = total_and_integration(u_form, v_form, u_coord, v_coord, v_max, xi, yi, dt)
        % Control Sum
        x_dot = u_form + u_coord;
        y_dot = v_form + v_coord;
        
        % VELOCITY LIMITER
        current_speed = sqrt(x_dot^2 + y_dot^2);
        if current_speed > v_max
            scale_factor = v_max / current_speed;
            x_dot = x_dot * scale_factor;
            y_dot = y_dot * scale_factor;
        end
        
        % Integration
        xi = xi + x_dot * dt;
        yi = yi + y_dot * dt;
end
%% --- NONHOLONOMIC CONTROL FUNCTIONS ---
% Function 12: Reference Dynamics
function [u1_r, u2_r, theta_r] = get_reference_dynamics(x_dot, y_dot, x_ddot, y_ddot, prev_theta)
    vel_sq = x_dot^2 + y_dot^2;
    
    % This prevents the robot from spinning wildly when the ghost stops.
    if vel_sq < 1e-6
        theta_r = prev_theta;
        u1_r = 0;
        u2_r = 0;
    else
        theta_r = atan2(y_dot, x_dot);
        u1_r = x_dot * cos(theta_r) + y_dot * sin(theta_r);
        u2_r = (x_dot * y_ddot - y_dot * x_ddot) / vel_sq;
    end
end
% Function 13: Tracking Error (Step B - Eq. 13)
function [e1, e2, e3] = get_tracking_error(gx, gy, theta_r, rx, ry, theta)
    x_telda = rx - gx;
    y_telda = ry - gy;
    theta_telda = theta - theta_r;
    theta_telda = atan2(sin(theta_telda), cos(theta_telda)); % Wrap angle
    
    e1 =  cos(theta) * x_telda + sin(theta) * y_telda;
    e2 = -sin(theta) * x_telda + cos(theta) * y_telda;
    e3 = theta_telda;
end
% Function 14: Control Law (Step C - Eq. 15)
function [u1, u2] = calculate_nonholonomic_control(e1, e2, e3, u1_r, u2_r, k1, k2, k3)
    % Linear Velocity
    u1 = -k1 * e1 + u1_r * cos(e3);
    
    % Sinc approximation
    if abs(e3) < 1e-3
        sinc_e3 = 1;
    else
        sinc_e3 = sin(e3) / e3;
    end
    
    % Angular Velocity (Eq 15)
    u2 = -u1_r * sinc_e3 * k3 * e2 - k2 * e3 + u2_r;
end
% Function 15: Nonholonomic Robot Dynamics (Eq. 6 & 7)
function [x_dot, y_dot, theta_dot] = nonholonomic_dynamics(u1, u2, theta)
    x_dot = u1 * cos(theta);
    y_dot = u1 * sin(theta);
    theta_dot = u2;
end
% Function 16: Plotter
function plot_simulation(gx, gy, rx, ry, rtheta, i)
    persistent ghost_trails robot_trails ghost_heads robot_heads robot_dirs
    
    colors = ['r', 'g', 'b', 'c', 'm'];
    c = colors(mod(i-1, length(colors))+1);
    
    % --- INITIALIZATION ---
    if length(ghost_heads) < i || isempty(ghost_heads{i}) || ~isvalid(ghost_heads{i})
        % Create Trails (Bottom Layer)
        ghost_trails{i} = animatedline('Color', c, 'LineStyle', '--', 'LineWidth', 1.0);
        robot_trails{i} = animatedline('Color', c, 'LineStyle', '-', 'LineWidth', 2.0);
        
        hold on;
        % Create Markers (Will be moved to Top Layer)
        ghost_heads{i} = plot(gx, gy, 'o', 'Color', c, 'MarkerSize', 5, 'LineWidth', 1);
        robot_heads{i} = plot(rx, ry, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
        robot_dirs{i} = plot([rx, rx], [ry, ry], 'k-', 'LineWidth', 1.5);
    else
        % --- UPDATE ANIMATION ---
        
        % 1. Add to Trails
        addpoints(ghost_trails{i}, gx, gy);
        addpoints(robot_trails{i}, rx, ry);
        
        % 2. Move Markers
        set(ghost_heads{i}, 'XData', gx, 'YData', gy);
        set(robot_heads{i}, 'XData', rx, 'YData', ry);
        
        % 3. Move Orientation Line
        len = 0.1;
        set(robot_dirs{i}, 'XData', [rx, rx + len*cos(rtheta)], ...
                           'YData', [ry, ry + len*sin(rtheta)]);
        
        % --- VISUALIZATION FIX: FORCE TO TOP ---
        % This ensures markers always float above the lines
        uistack(ghost_heads{i}, 'top');
        uistack(robot_heads{i}, 'top');
        uistack(robot_dirs{i}, 'top');
    end
    
    if i == 5 
        drawnow limitrate;
    end
end