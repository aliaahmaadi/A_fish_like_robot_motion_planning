clear; clc;

% ====================
% Parameters
% ====================
m = 39.9;           % Mass of the robot
I = 0.1;            % Moment of inertia
b = 0.52;           % Distance from center of mass to contact point
c = 29.2;           % Viscous dissipation coefficient
dt = 0.1;           % Time step
T = 150;            % Total simulation time
N = T / dt;         % Number of time steps

% Sinusoidal term parameters for tau_total
A = 1.5;            % Amplitude of sinusoidal torque input
w0 = 1.5;           % Frequency for sinusoidal input

% Control gain for heading error
k_heading = 1.0;    % Gain to convert heading error into desired angular velocity

% Target position
Xd = 10;
Yd = 10;

% Initial state [x, y, theta, ux, omega]
x0 = 0; y0 = 0; theta0 = 0; ux0 = 0; omega0 = 0;
state = [x0; y0; theta0; ux0; omega0];

% Obstacle positions and safe distance
obs = [4, 4; 8, 4; 6, 6];  % Obstacle positions (x, y)
dsafe = 2;                % Safe distance from obstacles

% QP constraint for tau_QP (corrective torque)
tauQP_limit = 0.2;
Aineq = [1; -1];          % 2x1 matrix for inequality constraints
bineq = [tauQP_limit; tauQP_limit];

% Pre-set options for quadprog
options = optimoptions('quadprog','Display','off');

% Pre-compute constant for angular dynamics denominator
a = 0.39;  % Used in angular dynamics

%% Preallocate arrays for data logging
timeArr = zeros(N,1);
xArr = zeros(N,1);
yArr = zeros(N,1);
thetaArr = zeros(N,1);
uxArr = zeros(N,1);
omegaArr = zeros(N,1);
tau_totalArr = zeros(N,1);
tauQPArr = zeros(N,1);
tauSinArr = zeros(N,1);
e_thetaArr = zeros(N,1);
distTargetArr = zeros(N,1);

%% Simulation Loop
for k = 1:N
    % Current simulation time
    t_sim = k * dt;
    timeArr(k) = t_sim;
    
    % Extract current state
    x = state(1);
    y = state(2);
    theta = state(3);
    ux = state(4);
    omega = state(5);
    
    % Log state values
    xArr(k) = x;
    yArr(k) = y;
    thetaArr(k) = theta;
    uxArr(k) = ux;
    omegaArr(k) = omega;
    distTargetArr(k) = norm([x - Xd; y - Yd]);
    
    % Compute desired heading based on target position
    thetad = atan2(Yd - y, Xd - x);
    
    % Obstacle avoidance adjustment
    for i = 1:size(obs, 1)
        obs_x = obs(i, 1);
        obs_y = obs(i, 2);
        dist = sqrt((x - obs_x)^2 + (y - obs_y)^2);
        if dist < dsafe
            % Compute repulsive angle away from the obstacle
            theta_obs = atan2(y - obs_y, x - obs_x);
            % Weighted sum of target direction and repulsion direction
            alpha = min(1, (dsafe - dist) / dsafe);
            thetad = (1 - alpha) * thetad + alpha * theta_obs;
        end
    end
    
    % Compute heading error (wrapped between -pi and pi)
    e_theta = atan2(sin(thetad - theta), cos(thetad - theta));
    e_thetaArr(k) = e_theta;
    % Desired angular velocity (proportional to heading error)
    thetadot_d = k_heading * e_theta;
    
    % Predict angular velocity without corrective torque
    d = omega + dt * (A * sin(w0 * t_sim) - m * b * omega * ux) / a;
    b0 = dt / a;
    
    % Compute QP parameters to minimize:
    %   J = (thetadot_d - (d + b0*tau_QP))^2
    e0 = thetadot_d - d;
    H_qp = 2 * b0^2;
    f_qp = -2 * b0 * e0;
    
    % Solve QP for tau_QP with constraint |tau_QP| <= tauQP_limit
    tau_QP = quadprog(H_qp, f_qp, Aineq, bineq, [], [], [], [], [], options);
    tauQPArr(k) = tau_QP;
    
    % Sinusoidal component of torque
    tauSin = A * sin(w0 * t_sim);
    tauSinArr(k) = tauSin;
    
    % Compute the total applied torque
    tau_total = tauSin + tau_QP;
    tau_totalArr(k) = tau_total;
    
    % ====================
    % Check if target is reached
    if norm([x - Xd; y - Yd]) < 0.1
        disp('Target reached!');
        % Freeze control: Set remaining states to target if desired
        state(1) = Xd;
        state(2) = Yd;
        % Optionally fill remaining arrays with final values
        xArr(k:end) = Xd;
        yArr(k:end) = Yd;
        break;
    end
    
    % ====================
    % Update Dynamics
    ux_dot = b * omega^2 - (c / m) * ux;
    omega_dot = (tau_total - m * b * omega * ux) / a;
    x_dot = ux * cos(theta) - (omega * b) * sin(theta);
    y_dot = ux * sin(theta) + (omega * b) * cos(theta);
    theta_dot = omega;
    
    % Euler integration for state update
    state(1) = state(1) + x_dot * dt;
    state(2) = state(2) + y_dot * dt;
    state(3) = state(3) + theta_dot * dt;
    state(4) = state(4) + ux_dot * dt;
    state(5) = state(5) + omega_dot * dt;
    
    % ====================
    % Plotting (Real-time animation)
    clf;
    hold on;
    plot(state(1), state(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    plot(Xd, Yd, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    for i = 1:size(obs, 1)
        plot(obs(i,1), obs(i,2), 'ks', 'MarkerSize', 10, 'LineWidth', 2);
        viscircles(obs(i,:), dsafe, 'Color', 'r', 'LineStyle', '--');
    end
    quiver(x, y, cos(thetad), sin(thetad), 2, 'r', 'LineWidth', 2);
    axis([-1 12 -1 12]);
    xlabel('X');
    ylabel('Y');
    title('Obstacle Avoidance with QP for Heading Control');
    grid on;
    drawnow;
end

% 1. Trajectory Plot (X-Y Path)
figure;
plot(xArr, yArr, 'b-', 'LineWidth', 2);
hold on;
plot(Xd, Yd, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
for i = 1:size(obs,1)
    viscircles(obs(i,:), dsafe, 'Color', 'r', 'LineStyle', '--');
end
xlabel('X');
ylabel('Y');
title('Trajectory of the Robot');
grid on;

% 2. State Variables
figure;
subplot(3,1,1);
plot(timeArr, xArr, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('X Position (m)');
grid on;
title('State Evolution');

subplot(3,1,2);
plot(timeArr, yArr, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Y Position (m)');
grid on;

subplot(3,1,3);
plot(timeArr, thetaArr, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Heading \theta (rad)');
grid on;

% 3. Control Inputs
figure;
subplot(3,1,1);
plot(timeArr, tauSinArr, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Sinusoidal Torque');
title('Control Inputs');
grid on;
subplot(3,1,2);
plot(timeArr, tauQPArr, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Corrective Torque \tau_{QP}');
grid on;
subplot(3,1,3);
plot(timeArr, tau_totalArr, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Total Torque \tau_{total}');
grid on;

% 4. Error Metrics vs Time
figure;
subplot(2,1,1);
plot(timeArr, e_thetaArr, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Heading Error (rad)');
title('Error Metrics');
grid on;
subplot(2,1,2);
plot(timeArr, distTargetArr, 'c-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Distance to Target');
grid on;

% 5. Phase Plot (ux vs. omega)
figure;
plot(uxArr, omegaArr, 'k-', 'LineWidth', 1.5);
xlabel('Linear Velocity (ux)');
ylabel('Angular Velocity (\omega)');
title('Phase Plot');
grid on;

