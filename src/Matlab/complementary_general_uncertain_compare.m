%% Complementary Filter for Angle Estimation with Alpha Comparison
clear; clc; close all;

%% Load Static Data
staticFile = "../../data/static/static_table_log_raw_4g_500dps.csv";
dynamicFile = "../../data/vision2/usb_pendulum_log_raw_4g_500dps_1.csv";

if ~isfile(staticFile), error('Data file does not exist: %s', staticFile); end
if ~isfile(dynamicFile), error('Data file does not exist: %s', dynamicFile); end

staticData = readtable(staticFile);
sAX = staticData{:, matches(staticData.Properties.VariableNames, 'AccelX')}';
sAY = staticData{:, matches(staticData.Properties.VariableNames, 'AccelY')}';
sAZ = staticData{:, matches(staticData.Properties.VariableNames, 'AccelZ')}';
sGX = staticData{:, matches(staticData.Properties.VariableNames, 'GyroX')}';

dynamicData = readtable(dynamicFile);
time = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'Timestamp')}';
aX = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'AccelX')}';
aY = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'AccelY')}';
aZ = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'AccelZ')}';
gX = dynamicData{:, matches(dynamicData.Properties.VariableNames, 'GyroX')}';

%% Calculate Theoretical Alpha
sThetaAccel = atan2(sAY, sqrt(sAX.^2 + sAZ.^2));
varAccelAngle = var(sThetaAccel);
varGyro = var(sGX);
dt = 0.008;
alpha_theoretical = varAccelAngle / (varAccelAngle + varGyro * dt);
fprintf('Theoretical alpha (angle fusion): %.3f\n', alpha_theoretical);

%% Initialize Variables for Comparison
alphas = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, alpha_theoretical]; % Alpha values to compare
n = length(time);
uncertainty_results = zeros(length(alphas), n);

%% Loop for Each Alpha
for i = 1:length(alphas)
    alpha = alphas(i);
    var_theta = zeros(1, n);
    var_theta(1) = varAccelAngle;
    for k = 2:n
        var_theta_gyro = var_theta(k-1) + varGyro * dt^2;
        var_theta(k) = alpha^2 * var_theta_gyro + (1 - alpha)^2 * varAccelAngle;
    end
    uncertainty_results(i, :) = sqrt(var_theta); % Store standard deviation (uncertainty)
end

%% Plot Uncertainty for Each Alpha
figure('Position', [100, 100, 1000, 800]);
hold on;
for i = 1:length(alphas)
    if alphas(i) == alpha_theoretical
        % Special label for theoretical alpha
        plot(time, uncertainty_results(i, :), 'DisplayName', ...
            sprintf('Alpha = %.3f (Theoretical)', alphas(i)), 'LineWidth', 2, 'LineStyle', '--');
    else
        plot(time, uncertainty_results(i, :), 'DisplayName', ...
            sprintf('Alpha = %.1f', alphas(i)), 'LineWidth', 1.5);
    end
end
xlabel('Time (s)', 'FontSize', 12);
ylabel('Uncertainty (rad)', 'FontSize', 12);
title('Uncertainty in Fused Angle for Different Alphas', 'FontSize', 14);
legend('FontSize', 10, 'Location', 'best');
grid on;
