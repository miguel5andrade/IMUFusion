% Load the data using readmatrix (automatically detects delimiters)
data = readmatrix('dados/teste_estatico_com_video_2min_de_movimento.TXT');

% Convert time from microseconds to seconds
time_seconds = data(:, 1) / 1000000;

% Convert accelerometer data from milli-g to m/s^2
accelData = data(:, 2:4) * 9.80665 / 1000; % Milli-g to m/s^2

% Convert gyro data from degrees/s to rad/s
gyroData = data(:, 5:7) * (pi / 180); % Degrees/s to rad/s

% Convert magnetometer data from microtesla to tesla
magData = data(:, 8:10) * 1e-6; % Microtesla to Tesla

% Compute sampling rate
sampleRate = 1 / mean(diff(time_seconds));

% Create the AHRS filter object
fusion = ahrsfilter('SampleRate', sampleRate);

% Fuse IMU data using the AHRS filter
orientation = fusion(accelData, gyroData, magData);

% Convert orientation (quaternions) to Euler angles
eulerAngles = eulerd(orientation, 'ZYX', 'frame'); % Convert to ZYX (yaw, pitch, roll) in degrees

% Display the Euler angles (Roll, Pitch, Yaw)
disp('Euler Angles (Roll, Pitch, Yaw) in degrees:');
disp(eulerAngles);

% Plot the Euler angles
figure;
plot(time_seconds, eulerAngles);
legend('Roll', 'Pitch', 'Yaw');
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('Euler Angles from IMU Data');


%% 
% Plotting results
figure;
subplot(3,1,1);
plot(time_seconds, eulerAnglesAHRS(:,3), 'r', 'DisplayName', 'Roll (IMU)');
title('Roll Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend;
grid on;

subplot(3,1,2);
plot(time_seconds, eulerAnglesAHRS(:,2), 'g', 'DisplayName', 'Pitch (IMU)');
title('Pitch Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend;
grid on;

subplot(3,1,3);
plot(time_seconds, eulerAnglesAHRS(:,1), 'b', 'DisplayName', 'Yaw (IMU)');
title('Yaw Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend;
grid on;

% Print some basic statistics
fprintf('IMU Filter Orientation Estimation Statistics:\n');
fprintf('Roll  - Mean: %.2f, Std: %.2f\n', mean(eulerAnglesIMU(:,3)), std(eulerAnglesIMU(:,3)));
fprintf('Pitch - Mean: %.2f, Std: %.2f\n', mean(eulerAnglesIMU(:,2)), std(eulerAnglesIMU(:,2)));
fprintf('Yaw   - Mean: %.2f, Std: %.2f\n', mean(eulerAnglesIMU(:,1)), std(eulerAnglesIMU(:,1)));


% Convert angular velocity (from rad/s to degrees/s)
angularVelocityDeg = angularVelocity * (180 / pi);

% Plot the filtered angular velocity
figure;
subplot(3,1,1);
plot(time_seconds, angularVelocityDeg(:,1), 'r', 'DisplayName', 'Gyro Roll (Filtered)');
title('Filtered Gyroscope Roll');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
legend;
grid on;

subplot(3,1,2);
plot(time_seconds, angularVelocityDeg(:,2), 'g', 'DisplayName', 'Gyro Pitch (Filtered)');
title('Filtered Gyroscope Pitch');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
legend;
grid on;

subplot(3,1,3);
plot(time_seconds, angularVelocityDeg(:,3), 'b', 'DisplayName', 'Gyro Yaw (Filtered)');
title('Filtered Gyroscope Yaw');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
legend;
grid on;

% Print basic statistics for angular velocity
fprintf('Filtered Angular Velocity Statistics (deg/s):\n');
fprintf('Roll  - Mean: %.2f, Std: %.2f\n', mean(angularVelocityDeg(:,1)), std(angularVelocityDeg(:,1)));
fprintf('Pitch - Mean: %.2f, Std: %.2f\n', mean(angularVelocityDeg(:,2)), std(angularVelocityDeg(:,2)));
fprintf('Yaw   - Mean: %.2f, Std: %.2f\n', mean(angularVelocityDeg(:,3)), std(angularVelocityDeg(:,3)));

%% 
% Load and process data as before
data = readmatrix('LAB1_3.txt');
time_seconds = data(:, 1) / 1000000;
dt = mean(diff(time_seconds));

% Convert accelerometer data from milli-g to m/s^2
accelData = data(:, 2:4) * 9.80665 / 1000; % Milli-g to m/s^2

% Convert gyro data from degrees/s to rad/s
gyroData = data(:, 5:7) * (pi / 180);

% Get orientation using imufilter
imuFilter = imufilter('SampleRate', 1/dt);
[orientation, angularVel] = imuFilter(accelData, gyroData);

% Remove gravity and get acceleration in world frame
% Initialize arrays
numSamples = length(time_seconds);
accel_world = zeros(numSamples, 3);
velocity = zeros(numSamples, 3);
position = zeros(numSamples, 3);

% Gravity vector
g = [0; 0; -9.80665];
% Simple ZUPT implementation
velocity_threshold = 0.05; % m/s
accel_threshold = 0.1; % m/s^2
% Process accelerometer data
for i = 1:numSamples
    % Rotate accelerometer readings to world frame
    if norm(accel_world(i,:)) < accel_threshold && norm(velocity(i,:)) < velocity_threshold
        velocity(i,:) = [0 0 0];
    end

    rotMatrix = rotmat(orientation(i), 'point');
    accel_body = accelData(i,:)';
    
    % Remove gravity and get acceleration in world frame
    accel_world(i,:) = (rotMatrix * accel_body + g)';
    
    % Integrate acceleration to get velocity
    if i > 1
        velocity(i,:) = velocity(i-1,:) + accel_world(i,:) * dt;
        
        % Integrate velocity to get position
        position(i,:) = position(i-1,:) + velocity(i,:) * dt;
    end
end

% Plot trajectories
figure;

% Determine the maximum range across all axes
maxRange = max([range(position(:,1)), range(position(:,2)), range(position(:,3))]);

% Calculate the limits for each axis
centerX = mean(position(:,1));
centerY = mean(position(:,2));
centerZ = mean(position(:,3));

xlim = [centerX - maxRange/2, centerX + maxRange/2];
ylim = [centerY - maxRange/2, centerY + maxRange/2];
zlim = [centerZ - maxRange/2, centerZ + maxRange/2];

% Plot the 3D trajectory
subplot(2,2,1);
plot3(position(:,1), position(:,2), position(:,3));
title('3D Position Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;

% Set axis limits and aspect ratio
axis([xlim, ylim, zlim]); % Apply equal limits to all axes
daspect([1 1 1]);          % Equal scaling for X, Y, and Z axes
axis vis3d;                % Fix aspect ratio while rotating
% Individual position components
subplot(2,2,2);
plot(time_seconds, position);
title('Position Components');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
grid on;

% Velocity components
subplot(2,2,3);
plot(time_seconds, velocity);
title('Velocity Components');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
grid on;

% Acceleration components (world frame)
subplot(2,2,4);
plot(time_seconds, accel_world);
title('Acceleration Components (World Frame)');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('X', 'Y', 'Z');
grid on;