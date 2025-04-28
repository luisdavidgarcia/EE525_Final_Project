% Kalman filter state estimation

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


%% Method #1 : Gyroscope integration for theta estimation (no filtering)
d_theta1 = gyroX;
theta1 = cumsum(d_theta1)*dt;

figure;
title('Method 1: Gyroscope Integration');
hold on;
plot(time, theta1, 'DisplayName', 'Theta');
plot(time, d_theta1, 'DisplayName', 'D Theta');
ylabel('State'); xlabel('Time [ms]');
legend;


%% Method #2 : Kalman filter gyroscope for theta estimation (pendulum model, no accelerometer)

% Define state transition matrix
Phi_c = [0, 1; -g/r, -b/m];
Phi = expm(Phi_c*dt);

% Define process error covariance (manually tune)
Q = eye(2);

% Define state-to-measurement matrix
H = [0, 1];

% Define measurement error covariance
R = varGyroX;

% Define measurement data
Z = gyroX;

% Setup filter
Horizon = length(time);

XHat = zeros(2, Horizon);
XHatMinus = zeros(2, Horizon);

P = zeros(2, 2, Horizon);
PMinus = zeros(2, 2, Horizon); PMinus(:, :, 1) = eye(2);

K = zeros(2, 1, Horizon);

% Execute filter
for k = 1:Horizon
    % Compute Kalman gain
    K(:, :, k) = PMinus(:, :, k)*H'/(H*PMinus(:, :, k)*H' + R);

    % Update estimate with measurement
    XHat(:, k) = XHatMinus(:, k) + K(:, :, k)*(Z(:, k) - H*XHatMinus(:, k));

    % Compute error covariance for updated estimate
    P(:, :, k) = (eye(2)-K(:, :, k)*H)*PMinus(:, :, k);

    % Project ahead
    if k < Horizon
        XHatMinus(:, k+1) = Phi*XHat(:, k);
        PMinus(:, :, k+1) = Phi*P(:, :, k)*Phi' + Q;
    end
end

figure;
title('Method 2: Kalman Filter Gyroscope (Pendulum Model, No Accelerometer)');
hold on;
plot(time, XHat(1, :), 'DisplayName', 'Theta');
plot(time, XHat(2, :), 'DisplayName', 'D Theta');
ylabel('State'); xlabel('Time [ms]');
legend;


%% Method #3 : Kalman filter gyroscope for theta estimation (gyroscope model, no accelerometer)

% Define state transition matrix
Phi = [1, dt; 0, 1];

% Define process error covariance (manually tune)
Q = eye(2);

% Define state-to-measurement matrix
H = [0, 1];

% Define measurement error covariance
R = varGyroX;

% Define measurement data
Z = gyroX;

% Setup filter
Horizon = length(time);

XHat = zeros(2, Horizon);
XHatMinus = zeros(2, Horizon);

P = zeros(2, 2, Horizon);
PMinus = zeros(2, 2, Horizon); PMinus(:, :, 1) = eye(2);

K = zeros(2, 1, Horizon);

% Execute filter
for k = 1:Horizon
    % Compute Kalman gain
    K(:, :, k) = PMinus(:, :, k)*H'/(H*PMinus(:, :, k)*H' + R);

    % Update estimate with measurement
    XHat(:, k) = XHatMinus(:, k) + K(:, :, k)*(Z(:, k) - H*XHatMinus(:, k));

    % Compute error covariance for updated estimate
    P(:, :, k) = (eye(2)-K(:, :, k)*H)*PMinus(:, :, k);

    % Project ahead
    if k < Horizon
        XHatMinus(:, k+1) = Phi*XHat(:, k);
        PMinus(:, :, k+1) = Phi*P(:, :, k)*Phi' + Q;
    end
end

figure;
title('Method 3: Kalman Filter Gyroscope (Gyroscope Model, No Accelerometer)');
hold on;
plot(time, XHat(1, :), 'DisplayName', 'Theta');
plot(time, XHat(2, :), 'DisplayName', 'D Theta');
ylabel('State'); xlabel('Time [ms]');
legend;


%% Method #4 : Kalman filter gyroscope for theta estimation (gyroscope model)

% Define state transition matrix
Phi = [1, dt; 0, 1];

% Define process error covariance (manually tune)
Q = eye(2);

% Define measurement data
theta = asin(accelZ/g);
Z = [theta; gyroX];

% Define state-to-measurement matrix
H = eye(2);

% Define measurement error covariance
varTheta = var(theta);
R = diag([varTheta, varGyroX]);

% Setup filter
Horizon = length(time);

XHat = zeros(2, Horizon);
XHatMinus = zeros(2, Horizon);

P = zeros(2, 2, Horizon);
PMinus = zeros(2, 2, Horizon); PMinus(:, :, 1) = eye(2);

K = zeros(2, 2, Horizon);

% Execute filter
for k = 1:Horizon
    % Compute Kalman gain
    K(:, :, k) = PMinus(:, :, k)*H'/(H*PMinus(:, :, k)*H' + R);

    % Update estimate with measurement
    XHat(:, k) = XHatMinus(:, k) + K(:, :, k)*(Z(:, k) - H*XHatMinus(:, k));

    % Compute error covariance for updated estimate
    P(:, :, k) = (eye(2)-K(:, :, k)*H)*PMinus(:, :, k);

    % Project ahead
    if k < Horizon
        XHatMinus(:, k+1) = Phi*XHat(:, k);
        PMinus(:, :, k+1) = Phi*P(:, :, k)*Phi' + Q;
    end
end

figure;
title('Method 4: Kalman Filter Gyroscope (Gyroscope Model)');
hold on;
plot(time, XHat(1, :), 'DisplayName', 'Theta');
plot(time, XHat(2, :), 'DisplayName', 'D Theta');
ylabel('State'); xlabel('Time [ms]');
legend;


%% Method #5 : Kalman filter gyroscope for theta estimation (pendulum model)

% Define state transition matrix
Phi_c = [0, 1; -g/r, -b/m];
Phi = expm(Phi_c*dt);

% Define process error covariance (manually tune)
Q = eye(2);

% Define measurement data
theta = asin(accelZ/g);
Z = [theta; gyroX];

% Define state-to-measurement matrix
H = eye(2);

% Define measurement error covariance
varTheta = var(theta);
R = diag([varTheta, varGyroX]);

% Setup filter
Horizon = length(time);

XHat = zeros(2, Horizon);
XHatMinus = zeros(2, Horizon);

P = zeros(2, 2, Horizon);
PMinus = zeros(2, 2, Horizon); PMinus(:, :, 1) = eye(2);

K = zeros(2, 2, Horizon);

% Execute filter
for k = 1:Horizon
    % Compute Kalman gain
    K(:, :, k) = PMinus(:, :, k)*H'/(H*PMinus(:, :, k)*H' + R);

    % Update estimate with measurement
    XHat(:, k) = XHatMinus(:, k) + K(:, :, k)*(Z(:, k) - H*XHatMinus(:, k));

    % Compute error covariance for updated estimate
    P(:, :, k) = (eye(2)-K(:, :, k)*H)*PMinus(:, :, k);

    % Project ahead
    if k < Horizon
        XHatMinus(:, k+1) = Phi*XHat(:, k);
        PMinus(:, :, k+1) = Phi*P(:, :, k)*Phi' + Q;
    end
end

figure;
title('Method 5: Kalman Filter Gyroscope (Pendulum Model)');
hold on;
plot(time, XHat(1, :), 'DisplayName', 'Theta');
plot(time, XHat(2, :), 'DisplayName', 'D Theta');
ylabel('State'); xlabel('Time [ms]');
legend;
