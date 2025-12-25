clc; clear all; close all;
%% Multi-Robot
a0 = 0.0; 
c0 = 0.0;
n = 4;
a = [1.5, 0.2, 0.05, 0.02];
b = [0.1, 0.05, -0.02, 0.01];
c = [-0.1, 0.05, 0.02, -0.01];
d = [1.0, -0.15, 0.05, -0.03];
% Generate Curves
[xt, yt] = get_efd_curve(a0, c0, a, b, c, d, n);
[f_ip] = efd_to_ip(a0, c0, a, b, c, d, n);
plot_efd(xt, yt);
plot_ip(f_ip);
title('Multi-Robot Simulation');

% Initial Conditions for N Robots
N = 5;                  
x = zeros(1, N);        
y = zeros(1, N);        
x(1) = 2; y(1) = 2;   
x(2) = 2.5; y(2) = 1.5;  
x(3) = 1; y(3) = 1.5;  
x(4) = 1.5; y(4) = 1;  
x(5) = 2; y(5) = 1;  

lambda = 3;
d_desired = 0.25;
k_shapeFormation = 2;
k_keepFormation = 1;

status = zeros(1, N);   
t = zeros(1, N);        
dt = 0.1;
direction = 1;
v_max = 0.1; 
omega = v_max/2;

traj_synced = zeros(1, N); 
t_ref = linspace(0, 2*pi, 200); 

% Main Loop
while true
    for i = 1:N
        % Check Status 
        xi = x(i); yi = y(i);
        [status(i), k] = check_curve_status(f_ip, x(i), y(i), k_shapeFormation, k_keepFormation);
        
        % Plot
        plot_robot_position(x(i), y(i), i);
        
        % Formation Control
        if status(i) == 0
            % PHASE 1: APPROACH (Implicit Polynomial)
            [u_form, v_form] = formation_ip(f_ip, x(i), y(i), lambda);
            traj_synced(i) = 0; 
        else
            % PHASE 2: MAINTAIN (Parametric EFD)
            
            % --- SYNCHRONIZATION STEP ---
            if traj_synced(i) == 0
                t(i) = get_nearest_t(xt, yt, x(i), y(i), t_ref);
                traj_synced(i) = 1; 
            end
            
            % Pass 'omega' to trajectory to scale velocity correctly
            [xt_star, yt_star, xt_star_dot, yt_star_dot] = trajectory(a0, c0, a, b, c, d, n, t(i), omega);
            
            [u_form, v_form] = formation_efd(xt_star, yt_star, xt_star_dot, yt_star_dot, x(i), y(i), lambda);
            
            % Increment time SLOWLY based on omega 
            t(i) = t(i) + direction * (dt * omega); 
            t(i) = mod(t(i), 2*pi);
        end
        
        % Coordination Control 
        [xp, yp, xq, yq, dip, diq] = nearest_two(x, y, i);
        [u_coord, v_coord] = coordination(xp, yp, xq, yq, dip, diq, d_desired, x(i), y(i), k);
        % Total Control 
        [xi, yi] = total_and_integration(u_form, v_form, u_coord, v_coord, v_max, xi, yi, dt);
        x(i) = xi;
        y(i) = yi;
    end
    
    pause(dt);
end

%% FUNCTION DEFINITIONS
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
    figure; hold on; axis equal; grid on; xlabel('X'); ylabel('Y');
    plot(xt, yt, 'b-', 'LineWidth', 2);
end
% Function 4: Plot IP
function plot_ip(f_ip)
    ax = axis; margin = 0.5; bounds = [ax(1)-margin, ax(2)+margin, ax(3)-margin, ax(4)+margin];
    fp = fimplicit(f_ip, bounds); fp.Color = 'r'; fp.LineStyle = ':'; fp.LineWidth = 3; fp.MeshDensity = 150; 
end
% Function 5a: Test Robot Status
function [status, k] = check_curve_status(f_ip, xi, yi, k_shapeFormation, k_keepFormation)
    tolerance = 0.1; val = f_ip(xi, yi);
    if abs(val) <= tolerance, status = 1; k = k_keepFormation; else, status = 0; k = k_shapeFormation; end
end
% Function 5b: Plot Robot 
function plot_robot_position(xi, yi, i)
    colors = ['r', 'g', 'b', 'c', 'm']; col_idx = mod(i-1, length(colors)) + 1; c = colors(col_idx);
    plot(xi, yi, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 6); drawnow limitrate;
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
% Function 10: Calculate Coordination Velocity
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