%% Extended Kalman Filter with Differential Drive Model
% Control input: [Δd, Δβ] where Δd is distance traveled and Δβ is heading change
% Measurement model: Range and bearing to beacons

clear;
close all;
clc;

%% Vehicle and Simulation Parameters
b = 0.5;                    % Track width (wheel separation) [m]
time_step = 0.2;            % Discrete time step [s]
simulation_time = 20;       % Total simulation time [s]
num_steps = simulation_time / time_step;

%% Define Trajectory
% You can change these to create different trajectories
trajectory_type = 'curve';  % Options: 'linear', 'circular', 'curve'

switch trajectory_type
    case 'linear'
        % Linear trajectory
        velocity_profile = 2.0 * ones(1, num_steps);        % Constant 2 m/s
        angular_velocity_profile = 0.0 * ones(1, num_steps); % Zero rotation
    case 'circular'
        % Circular/Arc trajectory
        velocity_profile = 1.0 * ones(1, num_steps); % Constant 1 m/s
        radius = 5; % 5m radius
        angular_velocity_profile = (velocity_profile / radius); % Constant angular velocity
    case 'curve'
        % Curved path with varying angular velocity
        velocity_profile = 1.5 * ones(1, num_steps); % Constant 1.5 m/s
        angular_velocity_profile = 0.2 * sin(2*pi*(0:num_steps-1)/num_steps); % Sinusoidal rotation [rad/s]
end


%% Process Noise (Motion Model Uncertainty)

% For EKF (in odometry space: Δd, Δβ)
process_noise_d = 0.01;      % Distance increment noise [m]
process_noise_beta = 0.02;   % Heading increment noise [rad]
Q = [process_noise_d^2, 0;
     0, process_noise_beta^2];

% Pre-compute standard deviations for efficiency
Q_std = sqrt(diag(Q));

%% Measurement Noise (Sensor Uncertainty)

% Range and bearing measurements to beacons
measurement_noise_range = 0.1;   % Range measurement noise [m]
measurement_noise_bearing = 0.05;  % Bearing measurement noise [rad]

% R matrix: [r1, phi1, r2, phi2, r3, phi3] - 3 beacons, 2 measurements each
R = diag([measurement_noise_range^2, measurement_noise_bearing^2, ...
          measurement_noise_range^2, measurement_noise_bearing^2, ...
          measurement_noise_range^2, measurement_noise_bearing^2]);

% Pre-compute standard deviations for efficiency
R_std = sqrt(diag(R));

%% Beacon Positions
beacons = [
    4  8;   % Beacon 1
    1  1;   % Beacon 2
   11  3    % Beacon 3
];
num_beacons = size(beacons, 1);
num_measurements = 2 * num_beacons;  % Range + bearing per beacon

%% Initial Conditions

% Initial state: start at origin, heading 45 degrees
true_state = [0; 0; pi/4];  % [x, y, theta]

% Initial state prediction (with error for EKF)
estimated_state = true_state + [0.5; 0.5; 0.1];  % Just deviate a little

% Initial state covariance: with uncertainty for the state vector
P = diag([0.5^2, 0.5^2, 0.1^2]);

%% Variables to show results
true_trajectory = zeros(3, num_steps);
estimated_trajectory = zeros(3, num_steps);
variance_history = zeros(3, num_steps);
control_history = zeros(2, num_steps);

%% Main Simulation Loop

for step = 1:num_steps
    % 1. Simulate ground truth robot motion: Differential drive model with
    % linear and angular velocity as control input, [v, ω]

    % Store previous state to compute actual displacement
    prev_state = true_state;

    % Differential drive control inputs: linear velocity (v) and angular velocity (ω)
    v = velocity_profile(step);
    omega = angular_velocity_profile(step);

    % Differential drive dynamics (no noise - perfect execution)
    % Note: v and ω could come from wheel velocities: v = (v_R + v_L)/2, ω = (v_R - v_L)/b

    % Use Euler method for integration (simpler, first-order)
    theta_k = true_state(3);

    % Update true state with Euler integration
    true_state = [
        true_state(1) + time_step * v * cos(theta_k);
        true_state(2) + time_step * v * sin(theta_k);
        theta_k + time_step * omega
    ];

    %% Compute Odometry Measurements [Δd, Δβ] from true motion - Vectorized
    % This is what the EKF actually observes (with noise)
    state_delta = true_state - prev_state;
    actual_control = [norm(state_delta(1:2)); state_delta(3)];

    % Add odometry measurement noise (vectorized)
    noisy_control = actual_control + Q_std .* randn(2, 1);
    control_history(:, step) = noisy_control;

    true_trajectory(:, step) = true_state;

    %% Generate Measurements (Range and Bearing to each beacon) - Vectorized
    % Compute all ranges and bearings at once
    dx_beacons = beacons(:,1) - true_state(1);
    dy_beacons = beacons(:,2) - true_state(2);

    % Range measurements
    true_ranges = sqrt(dx_beacons.^2 + dy_beacons.^2);

    % Bearing measurements (relative to robot heading)
    true_bearings = atan2(dy_beacons, dx_beacons);
    relative_bearings = true_bearings - true_state(3);

    % Range and bearing measurements: [r1, phi1, r2, phi2, r3, phi3]
    true_measurements = zeros(num_measurements, 1);
    true_measurements(1:2:end) = true_ranges;
    true_measurements(2:2:end) = relative_bearings;

    % Add measurement noise (using pre-computed R_std)
    measurements = true_measurements + R_std .* randn(num_measurements, 1);

    % Normalize bearing angles to [-pi, pi]
    measurements(2:2:end) = atan2(sin(measurements(2:2:end)), cos(measurements(2:2:end)));

    %% EKF PREDICTION STEP
    % Control input: u = [Δd, Δβ]
    delta_d_hat = noisy_control(1);
    delta_beta_hat = noisy_control(2);
    theta_k = estimated_state(3);

    % Predicted state using midpoint odometry model
    theta_mid = theta_k + delta_beta_hat / 2;
    predicted_state = [
        estimated_state(1) + delta_d_hat * cos(theta_mid);
        estimated_state(2) + delta_d_hat * sin(theta_mid);
        theta_k + delta_beta_hat
    ];

    % Compute Jacobian of discrete transition function with respect to state
    % f(x,y,θ) = [x + Δd*cos(θ + Δβ/2); y + Δd*sin(θ + Δβ/2); θ + Δβ]
    A = [
        1, 0, -delta_d_hat * sin(theta_mid);
        0, 1,  delta_d_hat * cos(theta_mid);
        0, 0,  1
    ];

    % Compute Jacobian with respect to control input u = [Δd, Δβ]
    % For midpoint model: ∂f/∂[Δd, Δβ]
    W = [
        cos(theta_mid),  -0.5 * delta_d_hat * sin(theta_mid);
        sin(theta_mid),   0.5 * delta_d_hat * cos(theta_mid);
        0,                1
    ];

    % Predict covariance
    P_predicted = A * P * A' + W * Q * W';

    %% EKF UPDATE STEP - Vectorized
    % Compute all predicted measurements at once
    dx_pred = beacons(:,1) - predicted_state(1);
    dy_pred = beacons(:,2) - predicted_state(2);
    dist_sq = dx_pred.^2 + dy_pred.^2;
    ranges_pred = sqrt(dist_sq);

    % Predicted bearings (relative to robot heading)
    predicted_bearings = atan2(dy_pred, dx_pred);
    predicted_relative_bearings = predicted_bearings - predicted_state(3);

    % Interleave predictions: [r1, phi1, r2, phi2, r3, phi3]
    predicted_measurements = zeros(num_measurements, 1);
    predicted_measurements(1:2:end) = ranges_pred;
    predicted_measurements(2:2:end) = predicted_relative_bearings;

    % Normalize bearing predictions to [-pi, pi]
    predicted_measurements(2:2:end) = atan2(sin(predicted_measurements(2:2:end)), ...
                                            cos(predicted_measurements(2:2:end)));

    %% Measurement Jacobian H (6x3 matrix)
    H = zeros(num_measurements, 3);
    for i = 1:num_beacons
        row_range = 2*i - 1;
        row_bearing = 2*i;

        % Range measurement Jacobian
        H(row_range, 1) = -dx_pred(i) / ranges_pred(i);  % ∂r/∂x
        H(row_range, 2) = -dy_pred(i) / ranges_pred(i);  % ∂r/∂y
        H(row_range, 3) = 0;                             % ∂r/∂θ

        % Bearing measurement Jacobian
        H(row_bearing, 1) = dy_pred(i) / dist_sq(i);     % ∂φ/∂x
        H(row_bearing, 2) = -dx_pred(i) / dist_sq(i);    % ∂φ/∂y
        H(row_bearing, 3) = -1;                          % ∂φ/∂θ
    end

    % Innovation (measurement residual) with angle normalization
    innovation = measurements - predicted_measurements;
    % Normalize bearing innovations to [-pi, pi]
    innovation(2:2:end) = atan2(sin(innovation(2:2:end)), cos(innovation(2:2:end)));

    % Innovation covariance
    S = H * P_predicted * H' + R;

    % Kalman gain
    K = P_predicted * H' / S;

    % Update state estimate
    estimated_state = predicted_state + K * innovation;

    % Normalize theta to [-pi, pi]
    estimated_state(3) = atan2(sin(estimated_state(3)), cos(estimated_state(3)));

    % Update covariance
    P = (eye(3) - K * H) * P_predicted;

    %% Store Results
    estimated_trajectory(:, step) = estimated_state;
    variance_history(:, step) = [P(1,1); P(2,2); P(3,3)];
end

%% Compute Estimation Errors
position_error = sqrt(sum((true_trajectory(1:2,:) - estimated_trajectory(1:2,:)).^2, 1));
angle_error = abs(true_trajectory(3,:) - estimated_trajectory(3,:));

%% Visualization
figure('Name', 'EKF with Range+Bearing Measurements', 'Position', [50 50 1400 900]);

% Plot 1: 2D Trajectory with Beacons
subplot(2,3,1);
hold on; grid on; axis equal;
plot(true_trajectory(1,:), true_trajectory(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True');
plot(estimated_trajectory(1,:), estimated_trajectory(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated');
plot(true_trajectory(1,1), true_trajectory(2,1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(beacons(:,1), beacons(:,2), 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k', 'DisplayName', 'Beacons');
xlabel('X [m]'); ylabel('Y [m]');
title(sprintf('Trajectory (%s)', trajectory_type));
legend('Ground truth', 'Estimated', 'Initial Position', 'Beacons', 'Location', 'best');

% Plot 2: X Position
subplot(2,3,2);
plot(true_trajectory(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(estimated_trajectory(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('X [m]');
title('X Position'); legend('Ground truth', 'Estimated');
grid on;

% Plot 3: Y Position
subplot(2,3,3);
plot(true_trajectory(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(estimated_trajectory(2,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Y [m]');
title('Y Position'); legend('Ground truth', 'Estimated');
grid on;

% Plot 4: Heading Angle
subplot(2,3,4);
plot(rad2deg(true_trajectory(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(rad2deg(estimated_trajectory(3,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Heading [deg]');
title('Heading Angle'); legend('Ground truth', 'Estimated');
grid on;

% Plot 5: Position Error
subplot(2,3,5);
plot(position_error, 'm-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Position Estimation Error');
grid on;

% Plot 6: Covariance
subplot(2,3,6);
semilogy(sqrt(variance_history(1,:)), 'r-', 'LineWidth', 1.5); hold on;
semilogy(sqrt(variance_history(2,:)), 'g-', 'LineWidth', 1.5);
semilogy(sqrt(variance_history(3,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Standard Deviation');
title('State Uncertainty');
legend('\sigma_x', '\sigma_y', '\sigma_\theta');
grid on;

%% Display Statistics
fprintf('\n========== EKF with range+bearing measurements - Results ==========\n');
fprintf('True Dynamics:          Differential Drive [v, ω]\n');
fprintf('EKF Control Input:      Odometry [Δd, Δβ]\n');
fprintf('Measurement Model:      Range + Bearing to %d beacons\n', num_beacons);
fprintf('Trajectory Type:        %s\n', trajectory_type);
fprintf('Track Width (b):        %.2f m\n', b);
fprintf('Time Step:              %.2f s\n', time_step);
fprintf('Simulation Time:        %.2f s\n', simulation_time);
fprintf('Number of Steps:        %d\n', num_steps);
fprintf('\nMeasurement Noise:\n');
fprintf('  Range Std Dev:        %.3f m\n', measurement_noise_range);
fprintf('  Bearing Std Dev:      %.3f rad (%.2f deg)\n', measurement_noise_bearing, rad2deg(measurement_noise_bearing));
fprintf('\nFinal Position Error:   %.3f m\n', position_error(end));
fprintf('Mean Position Error:    %.3f m\n', mean(position_error));
fprintf('Max Position Error:     %.3f m\n', max(position_error));
fprintf('Final Angle Error:      %.3f deg\n', rad2deg(angle_error(end)));
fprintf('\nFinal Std Dev (x):      %.4f m\n', sqrt(variance_history(1,end)));
fprintf('Final Std Dev (y):      %.4f m\n', sqrt(variance_history(2,end)));
fprintf('Final Std Dev (theta):  %.4f rad\n', sqrt(variance_history(3,end)));
fprintf('=================================================================\n\n');
