classdef EKFLandmarks < handle
    % EKFLandmarks - Extended Kalman Filter for Landmark-based Localization
    properties
        x           % State estimate [x; y; theta] (3x1)
        P           % State covariance matrix (3x3)
        landmarks   % Landmark positions [x, y] (Nx2)
        Q           % Process noise covariance [Δd; Δβ] (2x2)
        R           % Measurement noise covariance per landmark (2x2 or 1x1)
        chi2_tresh  % χ2 threshold [for 2-DOF, 95% (5.991), 99% (9.210)]
    end

    methods
        function obj = EKFLandmarks(initial_state, initial_covariance, ...
                                    landmarks, process_noise_cov, ...
                                    measurement_noise_cov, ...
                                    chi2_tresh)
            % Constructor
            arguments
                initial_state
                initial_covariance
                landmarks
                process_noise_cov
                measurement_noise_cov
                chi2_tresh (1,1) double {mustBePositive} = 9.21
            end

            obj.x = initial_state(:);
            obj.P = initial_covariance;
            obj.landmarks = landmarks;
            obj.Q = process_noise_cov;
            obj.R = measurement_noise_cov;
            obj.chi2_tresh = chi2_tresh;
        end

        function [matched_ids, matched_obs] = associate_landmarks(obj, observations)
            % associate_landmarks - Match observations to map landmarks
            %
            % Inputs:
            %   observations: Mx2 [range, bearing] or Mx1 [bearing] in robot frame
            %
            % Outputs:
            %   matched_ids: Kx1 vector of landmark indices (K <= M)
            %   matched_obs: Kx2 or Kx1 matched observations

            num_obs = size(observations, 1);
            has_range = size(observations, 2) == 2;

            matched_ids = [];
            matched_obs = [];

            % Predict what each landmark should look like from current pose
            num_landmarks = size(obj.landmarks, 1);
            dx_all = obj.landmarks(:,1) - obj.x(1);
            dy_all = obj.landmarks(:,2) - obj.x(2);
            ranges_pred = sqrt(dx_all.^2 + dy_all.^2);
            bearings_world = atan2(dy_all, dx_all);
            bearings_pred = bearings_world - obj.x(3);
            bearings_pred = atan2(sin(bearings_pred), cos(bearings_pred));

            % For each observation, find the best matching landmark
            for i = 1:num_obs
                z_obs = observations(i, :)';

                min_distance = inf;
                best_idx = -1;

                % Compare against all landmarks
                for j = 1:num_landmarks
                    if has_range
                        % Range-bearing
                        z_pred = [ranges_pred(j); bearings_pred(j)];
                        innovation = z_obs - z_pred;
                        innovation(2) = atan2(sin(innovation(2)), cos(innovation(2)));

                        dist = sqrt(innovation' / obj.R * innovation);
                    else
                        % Bearing-only
                        z_pred = bearings_pred(j);
                        innovation = z_obs - z_pred;
                        innovation = atan2(sin(innovation), cos(innovation));
                        dist = abs(innovation) / sqrt(obj.R);
                    end

                    if dist < min_distance
                        min_distance = dist;
                        best_idx = j;
                    end
                end

                % Accept match if within threshold (in Mahalanobis distance)
                if min_distance < obj.chi2_tresh  % Conservative threshold
                    matched_ids = [matched_ids; best_idx];
                    matched_obs = [matched_obs; observations(i, :)];
                end
            end

            % Ensure unique matches (each landmark matched at most once)
            if ~isempty(matched_ids)
                [unique_ids, first_occurrence] = unique(matched_ids, 'stable');
                matched_ids = unique_ids;
                matched_obs = matched_obs(first_occurrence, :);
            end

            % Debug output (can be removed later)
            if ~isempty(matched_ids)
                fprintf('Association: %d observations -> %d matches: [%s]\n', ...
                        num_obs, length(matched_ids), num2str(matched_ids'));
            else
                fprintf('Association: %d observations -> 0 matches\n', num_obs);
            end
        end

        function predict(obj, delta_d, delta_theta)
            % predict - EKF prediction step using odometry
            theta_k = obj.x(3);

            % Midpoint odometry model
            theta_mid = theta_k + delta_theta / 2;
            obj.x = [
                obj.x(1) + delta_d * cos(theta_mid);
                obj.x(2) + delta_d * sin(theta_mid);
                theta_k + delta_theta
            ];

            % Normalize heading angle
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            % Jacobian of transition function with respect to state
            % f(x,y,θ) = [x + Δd*cos(θ + Δβ/2); y + Δd*sin(θ + Δβ/2); θ + Δβ]
            A = [
                1, 0, -delta_d * sin(theta_mid);
                0, 1,  delta_d * cos(theta_mid);
                0, 0,  1
            ];

            % Jacobian with respect to odometry input u = [Δd; Δβ]
            W = [
                cos(theta_mid),  -0.5 * delta_d * sin(theta_mid);
                sin(theta_mid),   0.5 * delta_d * cos(theta_mid);
                0,                1
            ];

            % Predict covariance
            obj.P = A * obj.P * A' + W * obj.Q * W';
        end

        function update(obj, measurements, landmark_ids)
            % update - EKF update step using landmark observations
            %
            % Inputs:
            %   measurements: Mx2 [range, bearing] or Mx1 [bearing] observations
            %   landmark_ids: Mx1 vector of known landmark IDs (optional)

            % Step 1: Data association (if IDs not provided)
            if nargin < 3 || isempty(landmark_ids)
                [matched_ids, matched_obs] = obj.associate_landmarks(measurements);

                % If no landmarks matched, skip update
                if isempty(matched_ids)
                    return;
                end
            else
                % IDs provided, use measurements directly
                matched_ids = landmark_ids(:);
                matched_obs = measurements;
            end

            % Determine if we have range or bearing-only
            has_range = (size(matched_obs, 2) == 2);

            % Step 2: Sequential update for each matched observation
            for i = 1:length(matched_ids)
                %landmark_id = matched_ids(i);
                landmark_id = find(obj.landmarks(:,3)==matched_ids(i));
                landmark_pos = obj.landmarks(landmark_id, :);

                % Compute prediction for this landmark
                dx = landmark_pos(1) - obj.x(1);
                dy = landmark_pos(2) - obj.x(2);
                dist_sq = dx^2 + dy^2;
                range_pred = sqrt(dist_sq);

                % Predicted bearing (relative to robot heading)
                bearing_world = atan2(dy, dx);
                bearing_pred = bearing_world - obj.x(3);
                bearing_pred = atan2(sin(bearing_pred), cos(bearing_pred));

                if has_range
                    % Range-and-bearing measurement for this landmark
                    z = matched_obs(i, :)';  % [r; phi]
                    z_pred = [range_pred; bearing_pred];

                    % Measurement Jacobian (2x3)
                    H = [
                        -dx / range_pred,  -dy / range_pred,   0;    % ∂r/∂[x,y,θ]
                        dy / dist_sq,      -dx / dist_sq,      -1    % ∂φ/∂[x,y,θ]
                    ];

                    % Innovation with angle normalization
                    innovation = z - z_pred;
                    innovation(2) = atan2(sin(innovation(2)), cos(innovation(2)));
                else
                    % Bearing-only measurement
                    z = matched_obs(i);
                    z_pred = bearing_pred;

                    % Measurement Jacobian (1x3)
                    H = [dy / dist_sq, -dx / dist_sq, -1];

                    % Innovation with angle normalization
                    innovation = z - z_pred;
                    innovation = atan2(sin(innovation), cos(innovation));
                end

                % Innovation covariance
                S = H * obj.P * H' + obj.R;

                % Kalman gain
                K = obj.P * H' / S;

                % Update state estimate
                obj.x = obj.x + K * innovation;

                % Normalize theta to [-pi, pi]
                obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

                % Update covariance
                obj.P = (eye(3) - K * H) * obj.P;
            end
        end
    end
end
