clear all; close all; clc; delete(imaqfind);

%% 1. NETWORK & ROS SETUP
% Adjust these IPs to match your setup
setenv('ROS_DOMAIN_ID','50');               
% setenv('ROS_IP','192.168.1.100'); % Your PC IP          
setenv("ROS_LOCALHOST_ONLY","0");
setenv("RMW_IMPLEMENTATION","rmw_fastrtps_cpp");

try delete(ros2node("matlab_controller")); catch; end
node = ros2node("matlab_controller");

% Create Publishers for N robots
N_ROBOTS = 1; 
robotPubs = cell(1, N_ROBOTS);
robotMsgs = cell(1, N_ROBOTS);

for i = 1:N_ROBOTS
    % Topic naming: '/cmd_vel' for 1 robot. For swarm use: sprintf('/tb3_%d/cmd_vel', i)
    topicName = sprintf('/cmd_vel'); 
    
    % Your custom publisher configuration
    robotPubs{i} = ros2publisher(node, topicName, "geometry_msgs/Twist", ...
        "Reliability", "reliable", "Durability", "volatile", "Depth", 10);
        
    robotMsgs{i} = ros2message(robotPubs{i});
end
disp('ROS 2 Network Ready.');

%% 2. CAMERA & VISION SETUP
camID = 2; % Adjust to your overhead camera ID
tagFamily = "tag36h11";
tagSize_mm = 172; 
tagSize_m  = tagSize_mm / 1000; 

% Calibration (Standard Webcams)
focalLength    = [600, 600]; 
principalPoint = [320, 240];
imageSize      = [480, 640];
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);

%% 3. CONTROL ALGORITHM SETUP (From Simulation)
n = input('Enter number of harmonics (n): ');
a0 = 0; c0 = 0;
direction = 1;
[a, b, c, d] = generate_curve(n);

% Generate Curve Data
[xt, yt] = get_efd_curve(a0, c0, a, b, c, d, n);
[f_ip] = efd_to_ip(a0, c0, a, b, c, d, n);

% Control Parameters (Tuned)
lambda = 3;             
d_desired = 0.25;       % Meters
k_shape = 2;            
k_keep = 1;             

k1 = 2.0; % Longitudinal
k2 = 5.0; % Orientation
k3 = 20;  % Lateral (Anti-Drift)

v_max_ghost = 0.15;             % 10 cm/s
omega_ghost = v_max_ghost/2;    
v_max_robot = 0.10;             % Turtlebot3 Max is ~0.22
w_max_robot = 1.00;             

% State Memory
% Arrays for N robots (Ghost states)
x_ghost = zeros(1, N_ROBOTS); 
y_ghost = zeros(1, N_ROBOTS);
theta_ghost = zeros(1, N_ROBOTS); % Not strictly used by ghost but good for debug

status = zeros(1, N_ROBOTS);   
t_param = zeros(1, N_ROBOTS);        
traj_synced = zeros(1, N_ROBOTS); 
t_ref = linspace(0, 2*pi, 200); 

prev_theta_r = zeros(1, N_ROBOTS);
prev_vx = zeros(1, N_ROBOTS);
prev_vy = zeros(1, N_ROBOTS);

T_origin_stable = [];   % Stores the filtered pose matrix
alpha_origin = 0.90;    % Smoothing factor (0.9 = very smooth, 0.1 = very reactive)

% Flag to snap ghost to robot on first detection
ghost_initialized = false(1, N_ROBOTS);

%% 4. START CAMERA
vid = videoinput('winvideo', camID); 
triggerconfig(vid, 'manual');
vid.FramesPerTrigger = 1;
vid.TriggerRepeat = Inf;
start(vid);

%% 4.5. PREVIEW LOOP (WAIT FOR USER)
disp('--- PREVIEW MODE ---');
disp('Check camera feed. Press [SPACE] on the figure window to START CONTROL.');

hFig = figure('Name', 'Master Control Station', 'NumberTitle', 'off');
set(hFig, 'CurrentCharacter', '0'); % Reset key buffer

is_running = false;

while ishandle(hFig) && ~is_running
    % 1. Acquire & Detect
    I = getsnapshot(vid);
    I_gray = rgb2gray(I);
    [id, loc, pose] = readAprilTag(I_gray, tagFamily, intrinsics, tagSize_m);
    
    % 2. Draw Origin & Curve (If Origin Found)
    idx0 = find(id == 0, 1);
    if ~isempty(idx0)
        % --- ORIGIN SMOOTHING LOGIC ---
        T_raw = pose(idx0).A;
        if isempty(T_origin_stable)
            T_origin_stable = T_raw;
        else
            % Low-pass filter the matrix elements to remove jitter
            T_origin_stable = (alpha_origin * T_origin_stable) + ((1 - alpha_origin) * T_raw);
        end
        T_origin = T_origin_stable; 
        % ------------------------------

        % Project Curve to Image using Smoothed Origin
        curveWorldPoints = [xt', yt', zeros(length(xt), 1)];
        curveImgPoints = world2img(curveWorldPoints, pose(idx0), intrinsics); % Use raw pose for projection to align with image, or T_origin
        
        % Draw Curve (Cyan)
        I = insertShape(I, "Line", curveImgPoints, "Color", "cyan", "LineWidth", 3, "Opacity", 0.8);
        
        % Draw Origin Axes (Using Raw Pose for visual accuracy on the tag itself)
        I = helperInsertXYZAxes(I, pose(idx0), intrinsics, 0.3); 
    else
        I = insertText(I, [20 20], 'WAITING FOR ORIGIN (TAG 0)...', 'BoxColor', 'red', 'FontSize', 24);
    end
    
    % 3. Draw Detected Robots (Green Boxes)
    for i = 1:length(id)
        if id(i) ~= 0
            I = insertShape(I, "Polygon", loc(:,:,i), "Color", "green", "LineWidth", 3);
            I = insertText(I, loc(1,:,i), sprintf("ID: %d", id(i)), "FontSize", 18, "BoxColor", "black", "TextColor", "white");
        end
    end
    
    % 4. Overlay Status Text
    I = insertText(I, [size(I,2)/2 - 150, 20], 'PREVIEW MODE: PRESS SPACE TO START', ...
        'BoxColor', 'yellow', 'TextColor', 'black', 'FontSize', 18);
    
    imshow(I); 
    drawnow;
    
    % 5. Check for Spacebar
    k = get(hFig, 'CurrentCharacter');
    if k == ' '
        is_running = true;
        disp('>>> SPACE PRESSED. STARTING CONTROL LOOP... <<<');
    end
end

if ~ishandle(hFig), return; end % Exit if user closed window

%% 5. MAIN REAL-TIME LOOP
disp('System Running. Visualizing on Camera Feed...');
hFig = figure('Name', 'Overhead Vision Control', 'NumberTitle', 'off');

% Timing for dt calculation
last_time = tic;

while ishandle(hFig)
    % 1. Time Step Calculation
    current_time = tic;
    dt = toc(last_time);
    last_time = current_time;
    
    % Prevent huge dt jumps if loop hangs
    if dt > 0.1, dt = 0.1; end 
    
    % 2. Acquire Image & Detect Tags
    I = getsnapshot(vid);
    I_gray = rgb2gray(I);
    [id, loc, pose] = readAprilTag(I_gray, tagFamily, intrinsics, tagSize_m);
    
    % 3. Find Origin (Tag 0)
    idx0 = find(id == 0, 1);
    
    if ~isempty(idx0)
        % --- ORIGIN SMOOTHING LOGIC ---
        T_raw = pose(idx0).A;
        if isempty(T_origin_stable)
            T_origin_stable = T_raw;
        else
            T_origin_stable = (alpha_origin * T_origin_stable) + ((1 - alpha_origin) * T_raw);
        end
        T_origin = T_origin_stable;
        % ------------------------------
        
        % VISUALIZATION: Draw the Reference EFD Curve relative to Tag 0
        % We map the generated xt, yt (World) to Image coordinates
        curveWorldPoints = [xt', yt', zeros(length(xt), 1)];
        % Note: For visualization alignment, using the raw pose often looks "tighter" 
        % on the image, but T_origin is used for robot calculation.
        curveImgPoints = world2img(curveWorldPoints, pose(idx0), intrinsics);
        I = insertShape(I, "Line", curveImgPoints, "Color", "cyan", "LineWidth", 3, "Opacity", 0.6);
        
        % 4. Loop Through Robots (Tags 1..N)
        for i = 1:N_ROBOTS
            tagID = i; % Assuming Robot 1 has Tag 1
            idxR = find(id == tagID, 1);
            
            if ~isempty(idxR)
                % --- A. GET REAL ROBOT STATE (rx, ry, rtheta) ---
                T_robot = pose(idxR).A;
                T_rel = T_origin \ T_robot; % Transform: Origin -> Robot
                
                rx = T_rel(1, 4);
                ry = T_rel(2, 4);
                eul = rotm2eul(T_rel(1:3, 1:3)); 
                rtheta = eul(1); % Yaw
                
                % --- B. INITIALIZATION (Snap Ghost to Robot) ---
                if ~ghost_initialized(i)
                    x_ghost(i) = rx;
                    y_ghost(i) = ry;
                    prev_theta_r(i) = rtheta;
                    ghost_initialized(i) = true;
                    disp(['Robot ' num2str(i) ' Detected & Initialized']);
                end
                
                % --- C. GHOST SIMULATION (Logic from Previous Code) ---
                % 1. Status Check
                [status(i), k] = check_curve_status(f_ip, x_ghost(i), y_ghost(i), k_shape, k_keep);
                
                % 2. Formation Control
                if status(i) == 0
                    [u_form, v_form] = formation_ip(f_ip, x_ghost(i), y_ghost(i), lambda);
                    traj_synced(i) = 0;
                else
                    if traj_synced(i) == 0
                        t_param(i) = get_nearest_t(xt, yt, x_ghost(i), y_ghost(i), t_ref);
                        traj_synced(i) = 1;
                    end
                    [xt_s, yt_s, xt_sd, yt_sd] = trajectory(a0, c0, a, b, c, d, n, t_param(i), omega_ghost);
                    [u_form, v_form] = formation_efd(xt_s, yt_s, xt_sd, yt_sd, x_ghost(i), y_ghost(i), lambda);
                    
                    % Update Parameter t
                    t_param(i) = mod(t_param(i) + dt * omega_ghost * direction, 2*pi);
                end
                
                % 3. Coordination (Needs all ghost positions, assumed available in arrays)
                [xp, yp, xq, yq, dip, diq] = nearest_two(x_ghost, y_ghost, i);
                [u_coord, v_coord] = coordination(xp, yp, xq, yq, dip, diq, d_desired, x_ghost(i), y_ghost(i), k);
                
                % 4. Ghost Dynamics & Saturation
                gx_dot = u_form + u_coord;
                gy_dot = v_form + v_coord;
                
                current_speed = sqrt(gx_dot^2 + gy_dot^2);
                if current_speed > v_max_ghost
                    scale = v_max_ghost / current_speed;
                    gx_dot = gx_dot * scale;
                    gy_dot = gy_dot * scale;
                end
                
                % 5. Integrate Ghost Position
                x_ghost(i) = x_ghost(i) + gx_dot * dt;
                y_ghost(i) = y_ghost(i) + gy_dot * dt;
                
                % --- D. NONHOLONOMIC CONTROL (Calculate v, w) ---
                % 1. Calculate Acceleration
                gx_ddot = (gx_dot - prev_vx(i)) / dt;
                gy_ddot = (gy_dot - prev_vy(i)) / dt;
                prev_vx(i) = gx_dot; prev_vy(i) = gy_dot;
                
                % 2. Reference Dynamics
                [u1_r, u2_r, theta_r] = get_reference_dynamics(gx_dot, gy_dot, gx_ddot, gy_ddot, prev_theta_r(i));
                prev_theta_r(i) = theta_r;
                
                % 3. Tracking Error (Uses Real Robot rx, ry vs Ghost x, y)
                [e1, e2, e3] = get_tracking_error(x_ghost(i), y_ghost(i), theta_r, rx, ry, rtheta);
                
                % 4. Control Law
                [u1, u2] = calculate_nonholonomic_control(e1, e2, e3, u1_r, u2_r, k1, k2, k3);
                
                % 5. Saturation
                if abs(u1) > v_max_robot, u1 = sign(u1) * v_max_robot; end
                if abs(u2) > w_max_robot, u2 = sign(u2) * w_max_robot; end
                
                % --- E. SEND TO ROBOT (ROS 2) ---
                robotMsgs{i}.linear.x = u1;
                robotMsgs{i}.angular.z = u2;
                send(robotPubs{i}, robotMsgs{i});
                
                % --- F. VISUALIZATION (Draw Ghost & Robot on Image) ---
                % Project Ghost (x_ghost, y_ghost) to Image
                ghostWorld = [x_ghost(i), y_ghost(i), 0];
                ghostImg = world2img(ghostWorld, pose(idx0), intrinsics);
                
                % Draw Ghost (Cyan Circle)
                I = insertShape(I, "FilledCircle", [ghostImg, 5], "Color", "cyan");
                % Draw Robot Tag (Green Box)
                I = insertShape(I, "Polygon", loc(:,:,i), "Color", "green", "LineWidth", 3);
                % Draw Velocities Text
                txt = sprintf("ID:%d v:%.2f w:%.2f", tagID, u1, u2);
                I = insertText(I, loc(1,:,i), txt, "FontSize", 18, "BoxColor", "black", "TextColor", "white");
                
            else
                % Robot lost? Stop it for safety
                robotMsgs{i}.linear.x = 0;
                robotMsgs{i}.angular.z = 0;
                send(robotPubs{i}, robotMsgs{i});
            end
        end
    else
        I = insertText(I, [50 50], "ORIGIN (TAG 0) NOT FOUND", "FontSize", 24, "BoxColor", "red");
    end
    
    imshow(I); 
    drawnow;
end

stop(vid); delete(vid);

%% --- HELPER FUNCTIONS ---

% Function 00: Random Curve Generation
function [a, b, c, d] = generate_curve(n)
    % We generate a shape centered at (0,0) first to get the shape coefficients
    N_points = 1000;
    t = linspace(0, 2*pi, N_points);
    
    % --- SIZE ADJUSTMENT ---
    % Reduced base radius from 1.5 to 0.6
    r = 0.6 * ones(size(t)); 
    
    % Add random "wobbles" 
    for k_rand = 2:5 % Reduced harmonic range for smoother shapes
        noise_mag = 0.1 * rand(); 
        phase = 2*pi * rand();    
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

% Function 1: Generate Parametric Points (EFD)
function [xt, yt] = get_efd_curve(a0, c0, a, b, c, d, n)
    t = linspace(0, 2*pi, 200); xt = a0 * ones(size(t)); yt = c0 * ones(size(t));
    for k = 1:n
        xt = xt + a(k)*cos(k*t) + b(k)*sin(k*t); yt = yt + c(k)*cos(k*t) + d(k)*sin(k*t);
    end
end

% Function 2: Convert to Implicit Polynomial
function [f_ip] = efd_to_ip(a0, c0, a, b, c, d, n)
    A = (a - 1i*b)/2; B = (a + 1i*b)/2; C = (c - 1i*d)/2; D = (c + 1i*d)/2;
    g = [fliplr(B), a0, A]; h = [fliplr(D), c0, C];
    deg = 2 * n; monos = [];
    for k = 0:deg, for q = 0:k, monos = [monos; k-q, q]; end, end
    width = 2*(n*deg) + 1; M = zeros(size(monos,1), width); center_idx = n*deg + 1;
    for i = 1:size(monos,1)
        p = monos(i,1); q = monos(i,2); res = 1;
        for k=1:p, res = conv(res, g); end, for k=1:q, res = conv(res, h); end
        l = length(res); M(i, center_idx-floor(l/2) : center_idx+floor(l/2)) = res;
    end
    null_vec = null([real(M), imag(M)]'); ip_coeffs = null_vec(:, end);
    f_ip = @(x,y) 0;
    for i = 1:length(ip_coeffs)
        p = monos(i,1); q = monos(i,2); coeff = ip_coeffs(i);
        f_ip = @(x,y) f_ip(x,y) + coeff * (x.^p) .* (y.^q);
    end
end

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
    xt_star = a0; yt_star = c0; xt_star_dot = 0; yt_star_dot = 0;
    for k = 1:n
        xt_star = xt_star + a(k)*cos(k*ti) + b(k)*sin(k*ti);
        yt_star = yt_star + c(k)*cos(k*ti) + d(k)*sin(k*ti);
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
    u_coord_p = 0; v_coord_p = 0; u_coord_q = 0; v_coord_q = 0;
    if dip > 0
        u_coord_p = k * (d_desired - dip) * (xi - xp) / dip;
        v_coord_p = k * (d_desired - dip) * (yi - yp) / dip;
    end
    if diq > 0
        u_coord_q = k * (d_desired - diq) * (xi - xq) / diq;
        v_coord_q = k * (d_desired - diq) * (yi - yq) / diq;
    end
    u_coord = u_coord_p + u_coord_q; v_coord = v_coord_p + v_coord_q;
end

% Function 11: Find nearest parametric time t
function t_val = get_nearest_t(curve_x, curve_y, robot_x, robot_y, t_vector)
    dx = curve_x - robot_x; dy = curve_y - robot_y; dists = dx.^2 + dy.^2;
    [~, min_idx] = min(dists); t_val = t_vector(min_idx);
end

% Function 12: Reference Dynamics
function [u1_r, u2_r, theta_r] = get_reference_dynamics(x_dot, y_dot, x_ddot, y_ddot, prev_theta)
    vel_sq = x_dot^2 + y_dot^2;
    if vel_sq < 1e-6
        theta_r = prev_theta; u1_r = 0; u2_r = 0;
    else
        theta_r = atan2(y_dot, x_dot);
        u1_r = x_dot * cos(theta_r) + y_dot * sin(theta_r);
        u2_r = (x_dot * y_ddot - y_dot * x_ddot) / vel_sq;
    end
end

% Function 13: Tracking Error
function [e1, e2, e3] = get_tracking_error(gx, gy, theta_r, rx, ry, theta)
    x_telda = gx - rx; y_telda = gy - ry;
    theta_telda = theta - theta_r;
    theta_telda = atan2(sin(theta_telda), cos(theta_telda)); 
    e1 =  cos(theta) * x_telda + sin(theta) * y_telda;
    e2 = -sin(theta) * x_telda + cos(theta) * y_telda;
    e3 = theta_telda;
end

% Function 14: Control Law 
function [u1, u2] = calculate_nonholonomic_control(e1, e2, e3, u1_r, u2_r, k1, k2, k3)
    % Linear Velocity:
    % e1 > 0 means Reference is ahead -> Drive Forward (Positive u1)
    u1 = k1 * e1 + u1_r * cos(e3);
    
    % Angular Velocity:
    % e2 > 0 means Reference is to the Left -> Turn Left (Positive u2)
    if abs(e3) < 1e-3
        sinc_e3 = 1;
    else
        sinc_e3 = sin(e3) / e3;
    end
    
    % Note the POSITIVE sign on the e2 term now (matches Ref-Robot logic)
    u2 = u1_r * sinc_e3 * k3 * e2 + k2 * e3 + u2_r;
end

% Function 15: Draw 3D Axes on Tag
function I = helperInsertXYZAxes(I, pose, intrinsics, axisLength)
    % Define World Points for 0, X-axis, Y-axis, Z-axis
    xyzWorld = [0 0 0; axisLength 0 0; 0 axisLength 0; 0 0 axisLength];
    
    % Project to Image Plane
    xyzImg = world2img(xyzWorld, pose, intrinsics);
    
    % Draw Lines: Origin->X (Red), Origin->Y (Green), Origin->Z (Blue)
    lines = [xyzImg(1,:) xyzImg(2,:);  % X-axis
             xyzImg(1,:) xyzImg(3,:);  % Y-axis
             xyzImg(1,:) xyzImg(4,:)]; % Z-axis
             
    I = insertShape(I, "Line", lines, "Color", ["red", "green", "blue"], "LineWidth", 5);
end
