% Kalman filter state estimation with control inputs

clear; clc; close all;


%% Compute error variance
% Define path to static capture data file
staticFilePath = "../../data/static/static_table_log_raw_4g_500dps.csv";

% Check if the file exists
if ~isfile(staticFilePath)
    error('Data file does not exist: %s', staticFilePath);
end

% Load static data
staticData = readtable(staticFilePath);

staticAccelX = staticData{:, matches(staticData.Properties.VariableNames, 'AccelX')}';
staticAccelY = staticData{:, matches(staticData.Properties.VariableNames, 'AccelY')}';
staticAccelZ = staticData{:, matches(staticData.Properties.VariableNames, 'AccelZ')}';

staticGyroX = staticData{:, matches(staticData.Properties.VariableNames, 'GyroX')}';
staticGyroY = staticData{:, matches(staticData.Properties.VariableNames, 'GyroY')}';
staticGyroZ = staticData{:, matches(staticData.Properties.VariableNames, 'GyroZ')}';

% Compute noise variances
varAccelX = var(staticAccelX);
varAccelY = var(staticAccelY);
varAccelZ = var(staticAccelZ);

varGyroX = var(staticGyroX);
varGyroY = var(staticGyroY);
varGyroZ = var(staticGyroZ);


%% Load swing data
% Define the path to dynamic capture data file
dataFilePath = "../../data/vision2/usb_pendulum_log_raw_4g_500dps_1.csv";

% Check if the file exists
if ~isfile(dataFilePath)
    error('Data file does not exist: %s', dataFilePath);
end

% Load table
data = readtable(dataFilePath);

% Parse table
time = data{:, matches(data.Properties.VariableNames, 'Timestamp')}';

accelX = data{:, matches(data.Properties.VariableNames, 'AccelX')}';
accelY = data{:, matches(data.Properties.VariableNames, 'AccelY')}';
accelZ = data{:, matches(data.Properties.VariableNames, 'AccelZ')}';

gyroX = data{:, matches(data.Properties.VariableNames, 'GyroX')}';
gyroY = data{:, matches(data.Properties.VariableNames, 'GyroY')}';
gyroZ = data{:, matches(data.Properties.VariableNames, 'GyroZ')}';


%% Define constants
dt = 0.008;  % Sample period
g = 9.80665; % Standard gravity
b = 0.0190;  % Drag coefficient (optimal)
m = 0.0837;  % Mass (optimal)
r = 0.3628;  % Radius (optimal)


%% Kalman Filter (with control input)

% Define state transition matrix
Phi = [1,0,-dt;
       0,0, -1;
       0,0,  1];

% Define control input to state matrix
B = [dt;
     1;
     0];

% Define control input
U = gyroX;

% Define control input noise covariance
Q = diag([varGyroX, varGyroX, 0]); % 1e-6 works well for bias variance

% Define state to measurement matrix
H = [1,0,0];

% Define measurement input
roll = atan2(accelY, sqrt(accelX.^2 + accelZ.^2));

Z = roll;

% Define measurement input noise covariance
R = var(roll);

% Setup filter
Horizon = length(time);

XHat = zeros(3, Horizon);
XHatMinus = zeros(3, Horizon);

P = zeros(3, 3, Horizon);
PMinus = zeros(3, 3, Horizon); PMinus(:, :, 1) = eye(3);

K = zeros(3, 1, Horizon);

% Execute filter
for k = 1:Horizon
    % Compute Kalman gain
    K(:, :, k) = PMinus(:, :, k)*H'/(H*PMinus(:, :, k)*H' + R);

    % Update estimate with measurement
    XHat(:, k) = XHatMinus(:, k) + K(:, :, k)*(Z(:, k) - H*XHatMinus(:, k));

    % Compute error covariance for updated estimate
    P(:, :, k) = (eye(3)-K(:, :, k)*H)*PMinus(:, :, k);

    % Project ahead
    if k < Horizon
        XHatMinus(:, k+1) = Phi*XHat(:, k) + B*U(:, k);
        PMinus(:, :, k+1) = Phi*P(:, :, k)*Phi' + Q;
    end
end

figure;
title('Kalman Filter Orientation Estimation');
hold on;
plot(time, XHat(1, :), 'DisplayName', 'Roll');
plot(time, XHat(2, :), 'DisplayName', 'D Roll');
ylabel('Rad'); xlabel('Time [ms]');
legend;

figure;
title('Kalman Filter Bias Estimation');
hold on;
plot(time, XHat(3, :), 'DisplayName', 'Gyro Roll Bias');
ylabel('Rad/s'); xlabel('Time [ms]');
legend;
