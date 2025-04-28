%% Complementary Filter for Angle Estimation
%% Setup
clear; clc; close all;

%% Calculate Ground Truth
% Define the path to the data file
dataFilePath = "../../data/vision2_analysis/usb_pendulum_video_1_analysis.csv";

% Check if the file exists
if ~isfile(dataFilePath)
    error('Data file does not exist: %s', dataFilePath);
end

% Read data from log file into a table
data = readtable(dataFilePath);

%% Setup Experimental Data
timeVision = data{:, matches(data.Properties.VariableNames, 'Timestamp')};
% Vision in seconds * 1000 to match IMU timestamps
timeVision = timeVision * 1000;
posX = data{:, matches(data.Properties.VariableNames, 'PosX')};
posY = data{:, matches(data.Properties.VariableNames, 'PosY')};
clickPosX = data{1, matches(data.Properties.VariableNames, 'ClkPosX')};
clickPosY = data{1, matches(data.Properties.VariableNames, 'ClkPosY')};

% Calculate ground truth angles
vec = [posX, posY] - [clickPosX, clickPosY];
ground_truth_theta = atan2(vec(:, 1), vec(:, 2));
ground_truth_theta = ground_truth_theta - mean(ground_truth_theta);

staticFile = "../../data/static/static_table_log_raw_4g_500dps.csv";
dynamicFile = "../../data/vision2/usb_pendulum_log_raw_4g_500dps_1.csv";

if ~isfile(staticFile), error('Data file does not exist: %s', staticFile); end
if ~isfile(dynamicFile), error('Data file does not exist: %s', dynamicFile); end

%% Load Static Data
staticData = readtable(staticFile);
sAX = staticData{:, matches(staticData.Properties.VariableNames, 'AccelX')}';
sAY = staticData{:, matches(staticData.Properties.VariableNames, 'AccelY')}';
sAZ = staticData{:, matches(staticData.Properties.VariableNames, 'AccelZ')}';
sGX = staticData{:, matches(staticData.Properties.VariableNames, 'GyroX')}';
sGY = staticData{:, matches(staticData.Properties.VariableNames, 'GyroY')}';
sGZ = staticData{:, matches(staticData.Properties.VariableNames, 'GyroZ')}';

%% Load Dynamic Data
dynamicData = readtable(dynamicFile);
time = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'Timestamp')}';
aX = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'AccelX')}';
aY = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'AccelY')}';
aZ = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'AccelZ')}';
gX = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'GyroX')}';
gY = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'GyroY')}';
gZ = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'GyroZ')}';


%% Calculate Theoretical Optimal Alpha and Beta

% Noise variance for accelerometer (angle from static data)
sThetaAccel = atan2(sAY, sqrt(sAX.^2 + sAZ.^2));
varAccelAngle = var(sThetaAccel);
varGyro = var(sGX);

% Theoretical alpha (angle fusion)
dt = 0.008;
theoretical_alpha = varAccelAngle / (varAccelAngle + varGyro * dt);
fprintf('Theoretical alpha (angle fusion): %.3f\n', theoretical_alpha);

alpha = 0.99; % Empirically optimized alpha
fprintf('Empirical alpha (angle fusion): %.3f\n', alpha);


%% Complementary Filter Setup
n = length(time);
theta = zeros(1, n);
theta_gyro = zeros(1, n);
theta_accel = zeros(1, n);

% Initial angle (from accelerometer)
theta(1) = atan2(aY(1), sqrt(aX(1)^2 + aZ(1)^2));

%% Loop through dynamic data
for k = 2:n
    % Gyroscope-based angle
    theta_gyro(k) = theta(k-1) + gX(k) * dt;

    % Accelerometer-based angle
    theta_accel(k) = atan2(aY(k), sqrt(aX(k)^2 + aZ(k)^2));

    % Complementary filter
    theta(k) = alpha * theta_gyro(k) + (1 - alpha) * theta_accel(k);
end

% Zero-mean the theta signal to match ground truth processing
theta = detrend(theta, 'linear');   % Removes linear trend

%% Initialize Uncertainty Tracking
var_theta = zeros(1, n);
var_theta(1) = varAccelAngle;

%% Loop through dynamic data (tracking uncertainties)
for k = 2:n
    % Gyroscope variance for angle (propagated over time)
    var_theta_gyro = var_theta(k-1) + varGyro * dt^2;

    % Variance of the fused angle (complementary filter)
    var_theta(k) = alpha^2 * var_theta_gyro + (1 - alpha)^2 * varAccelAngle;
end

figure('Position', [100, 100, 1000, 800]);
plot(time, sqrt(var_theta), 'DisplayName', 'Uncertainty in Fused Angle (std dev)', 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Uncertainty (rad)', 'FontSize', 12);
title('Uncertainty in Fused Angle Over Time (Complementary Filter)', 'FontSize', 14);
grid on;
legend('FontSize', 10, 'Location', 'best');


% Plot comparison
start = 1755;

short_theta = theta(start:end);  % Also truncate theta data

% Calculate the time offset needed to align the signals
% You can do this by finding the delay between the start of oscillation
time_offset = 14300;  % This is an estimate based on your plot, you may need to adjust

% Adjust the IMU timestamps by subtracting the offset
short_time = time(start:end) - time_offset;

% Adjust end of time vision
last_time = 946;
timeVision = timeVision(1:last_time);
ground_truth_theta = ground_truth_theta(1:last_time);

% Calculate required sampling rate to get exactly 946 points
n_samples = length(short_theta);
sample_rate = floor(n_samples/last_time);  % This will give us how many points to skip

% Resample short_theta and short_time to match ground truth length
indices = round(linspace(1, length(short_theta), last_time));
short_theta = short_theta(indices);
short_time = short_time(indices);



% Then in your plotting section:
figure('Position', [100, 100, 1000, 800]);
plot(short_time, short_theta, 'DisplayName', 'Fused Angle', 'LineWidth', 2.0);
hold on;
plot(timeVision, ground_truth_theta, '--', 'DisplayName', 'Observed', 'LineWidth', 1.5);
legend('FontSize', 10, 'Location', 'best');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angle (rad)', 'FontSize', 12);
title('Comparison of Fused Angle vs. Observed', 'FontSize', 14);
grid on;

% Calculate MSE and RMSE once
% truncate theta
MSE = mean((short_theta - ground_truth_theta').^2);

% Print results once
fprintf('Mean Squared Error: %.6f rad^2\n', MSE);

% Additional plot for theoretical alpha comparison
theta_theoretical = zeros(1, n);
theta_theoretical(1) = atan2(aY(1), sqrt(aX(1)^2 + aZ(1)^2));

% Loop for theoretical alpha
for k = 2:n
    theta_gyro_theoretical = theta_theoretical(k-1) + gX(k) * dt;
    theta_accel_theoretical = atan2(aY(k), sqrt(aX(k)^2 + aZ(k)^2));
    theta_theoretical(k) = theoretical_alpha * theta_gyro_theoretical + ...
        (1 - theoretical_alpha) * theta_accel_theoretical;
end

% Zero-mean and detrend
theta_theoretical = detrend(theta_theoretical, 'linear');

% Resample theoretical data to match ground truth length
short_theta_theoretical = theta_theoretical(start:end);
short_theta_theoretical = short_theta_theoretical(indices);

% Create new figure for comparison
figure('Position', [100, 100, 1000, 800]);
plot(short_time, short_theta_theoretical, '-', 'LineWidth', 2.0, ...
    'DisplayName', sprintf('Theoretical (α = %.3f)', theoretical_alpha));
hold on;
plot(timeVision, ground_truth_theta, '--', 'LineWidth', 1.5, ...
    'DisplayName', 'Observed');
legend('FontSize', 10, 'Location', 'best');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angle (rad)', 'FontSize', 12);
title('Comparison of Theoretical vs Empirical Alpha Performance', 'FontSize', 14);
grid on;

% Calculate MSE for theoretical alpha
MSE_theoretical = mean((short_theta_theoretical - ground_truth_theta').^2);
fprintf('Theoretical Alpha MSE: %.6f rad^2\n', MSE_theoretical);
