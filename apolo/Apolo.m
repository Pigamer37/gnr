classdef Apolo < handle
    % Apolo - Wrapper class for Apolo simulator robot interface
    %
    % This class encapsulates all interactions with the Apolo simulator,
    % providing a clean interface for robot control, sensor access, and
    % state management.
    %
    % Example:
    %   robot = Apolo("MazeRunner.xml", "Pioneer3ATSim", "LMS100");
    %   robot.reset([0, 0, 0]);
    %
    %   scan = robot.getLaserScan();
    %   robot.move([1.0, 0.5], 0.1);  % v=1.0 m/s, omega=0.5 rad/s, dt=0.1s
    %   [delta_d, delta_theta] = robot.getOdometry();

    properties (SetAccess = private)
        robotName       % Robot name in simulator
        laserName       % LiDAR sensor name
        worldXML        % Parsed world XML structure

        % Current state
        position        % [x, y, z, theta] - full 4D pose from simulator
        state           % [x, y, theta] - 3D state for control/estimation
        prev_odom       % Previous odometry reading

        % Sensor data (cached)
        laser_scan      % Latest LiDAR scan [range, bearing]
        landmarks       % Latest landmark measurements
    end

    methods
        function obj = Apolo(worldXMLFile, robotName, laserName)
            % Apolo Constructor - Initialize robot interface
            %
            % Inputs:
            %   worldXMLFile - Path to world XML configuration file
            %   robotName    - Name of robot in simulator
            %   laserName    - Name of LiDAR sensor
            %
            % Example:
            %   robot = Apolo("MazeRunner.xml", "Pioneer3ATSim", "LMS100");

            arguments
                worldXMLFile (1,1) string
                robotName (1,1) string
                laserName (1,1) string
            end

            obj.worldXML = readstruct(worldXMLFile, "FileType", "xml");
            % Read this from file
            obj.robotName = char(robotName);
            obj.laserName = char(laserName);
            obj.updateState();
        end

        function reset(obj, start_pose)
            % reset - Reset robot to initial pose
            %
            % Inputs:
            %   start_pose - [x, y, theta] initial pose
            %
            % Example:
            %   robot.reset([1.0, 2.0, pi/4]);

            arguments
                obj
                start_pose (1,3) double
            end

            if apoloPlaceMRobot(obj.robotName, [start_pose(1), start_pose(2), 0], start_pose(3)) ~= 1
                error("Apolo:PlacementError", "Error placing %s at position [%.2f, %.2f, %.2f]", ...
                    obj.robotName, start_pose(1), start_pose(2), start_pose(3));
            end

            apoloUpdate();

            apoloResetOdometry(obj.robotName, start_pose);
            obj.updateState();
        end

        function updateState(obj)
            % updateState - Update current robot state from simulator
            %
            % This is called automatically after move() and reset().
            % You can call it manually if needed.

            obj.position = apoloGetLocationMRobot(obj.robotName);
            obj.state = [obj.position(1); obj.position(2); obj.position(4)];
        end

        function move(obj, control, dt, pause_time)
            % move - Move robot with control input for specified time
            %
            % Inputs:
            %   control     - [v, omega] linear and angular velocity
            %   dt          - Time step duration [s]
            %   pause_time  - Optional pause for visualization [s] (default: 0)
            %
            % Example:
            %   robot.move([1.0, 0.5], 0.1);  % Move with v=1.0, omega=0.5 for 0.1s
            %   robot.move([0.5, 0.0], 0.1, 0.05);  % With 50ms pause

            arguments
                obj
                control (1,2) double
                dt (1,1) double {mustBePositive}
                pause_time (1,1) double {mustBeNonnegative} = 0
            end

            obj.prev_odom = apoloGetOdometry(obj.robotName);
            apoloMoveMRobot(obj.robotName, control, dt);
            apoloUpdate();

            if pause_time > 0
                pause(pause_time);
            end

            obj.updateState();
        end

        function [delta_d, delta_theta] = getOdometry(obj)
            % getOdometry - Get odometry increment since last move
            %
            % Returns:
            %   delta_d     - Distance increment [m]
            %   delta_theta - Heading increment [rad]
            %
            % Note: Only valid after calling move() at least once
            %
            % Example:
            %   robot.move([1.0, 0.0], 0.1);
            %   [delta_d, delta_theta] = robot.getOdometry();

            if isempty(obj.prev_odom)
                warning('Apolo:NoOdometry', 'No previous odometry available. Call move() first.');
                delta_d = 0;
                delta_theta = 0;
                return;
            end

            [delta_d, delta_theta] = apolo_odometry(obj.robotName, obj.prev_odom);
        end

        function scan = getLaserScan(obj)
            % getLaserScan - Get LiDAR scan in [range, bearing] format
            %
            % Returns:
            %   scan - Nx2 matrix where each row is [range, bearing]
            %          range: distance to obstacle [m]
            %          bearing: angle relative to robot heading [rad]
            %
            % Example:
            %   scan = robot.getLaserScan();
            %   valid_points = scan(~isnan(scan(:,1)), :);

            scan_raw = apoloGetLaserData(obj.laserName);
            b = size(scan_raw);
            ang = 0:b(2)-1;

            if b(2) > 181
                % LMS200 or similar: 270 degree scan
                ang = (ang - 270) * (1.5 * pi / b(2));
            else
                % LMS100: 180 degree scan
                scan_raw = scan_raw(1:180);
                ang = 0:179;
                ang = (ang - 90) * (pi / b(2));
            end

            scan = [scan_raw(:), ang(:)];
            obj.laser_scan = scan;
        end

        function [measurements, ids] = getLandmarkMeasurements(obj)
            % getLandmarkMeasurements - Get range-bearing to visible landmarks
            %
            % Returns:
            %   measurements - Mx2 matrix of [range, bearing] for M visible landmarks
            %   ids          - 1xM vector of landmark IDs
            %
            % Example:
            %   [measurements, ids] = robot.getLandmarkMeasurements();
            %   if ~isempty(measurements)
            %       for i = 1:length(ids)
            %           fprintf('Landmark %d: range=%.2f, bearing=%.2f\n', ...
            %               ids(i), measurements(i,1), measurements(i,2));
            %       end
            %   end

            lm_meas = apoloGetLaserLandMarks(obj.laserName);
            ids = lm_meas.id;

            if isempty(ids)
                measurements = [];
                obj.landmarks = [];
                return;
            end

            % Format as [range, bearing] pairs
            measurements = [lm_meas.distance', lm_meas.angle'];

            % Normalize bearing angles to [-pi, pi]
            measurements(:, 2) = atan2(sin(measurements(:, 2)), cos(measurements(:, 2)));

            obj.landmarks = measurements;
        end

        function beacons = getBeaconPositions(obj)
            % getBeaconPositions - Extract beacon positions from world XML
            %
            % Returns:
            %   beacons - Nx3 matrix where each row is [x, y, id]
            %             Empty matrix if no beacons in world
            %
            % Example:
            %   beacons = robot.getBeaconPositions();
            %   num_beacons = size(beacons, 1);

            % Handle case where World doesn't have LandMark field
            if ~isfield(obj.worldXML.World, 'LandMark')
                beacons = [];
                return;
            end

            num_beacons = size(obj.worldXML.World.LandMark, 2);

            if num_beacons == 0
                beacons = [];
                return;
            end

            beacons = zeros(num_beacons, 3);
            for i = 1:num_beacons
                coords = split(extractBetween(obj.worldXML.World.LandMark(i).position, "{", "}"), ",");
                beacons(i, :) = [str2double(coords(1)), str2double(coords(2)), ...
                                 obj.worldXML.World.LandMark(i).mark_idAttribute];
            end
            
            % Sort by ID to ensure array index matches landmark ID
            beacons = sortrows(beacons, 3);
        end

        function x = getState(obj)
            % getState - Get current 2D state [x; y; theta]
            %
            % Returns:
            %   x - 3x1 state vector [x; y; theta]
            %
            % Example:
            %   x = robot.getState();
            %   fprintf('Position: [%.2f, %.2f], Heading: %.2f deg\n', ...
            %       x(1), x(2), rad2deg(x(3)));

            x = obj.state;
        end

        function pos = getPosition(obj)
            % getPosition - Get full 3D position [x, y, z, theta]
            %
            % Returns:
            %   pos - 1x4 vector [x, y, z, theta]
            %
            % Example:
            %   pos = robot.getPosition();

            pos = obj.position;
        end
    end
end
