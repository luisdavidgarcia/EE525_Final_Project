%% Refine Alpha Search in Range 0.8 to 1.0
clear; clc; close all;

% Load Static Data (Assume already loaded as in previous code)
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

% Theoretical Alpha (angle fusion)
sThetaAccel = atan2(sAY, sqrt(sAX.^2 + sAZ.^2));
varAccelAngle = var(sThetaAccel);
varGyro = var(sGX);
dt = 0.008;
alpha_theoretical = varAccelAngle / (varAccelAngle + varGyro * dt);
fprintf('Theoretical alpha (angle fusion): %.3f\n', alpha_theoretical);

%% Refined Alpha Range
alphas = 0.9:.01:1.0; % Narrow range with finer steps
alphas =[alphas, alpha_theoretical]; % Add theoretical alpha
n = length(time);
uncertainty_results = zeros(length(alphas), n);

% Loop for Each Refined Alpha
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

%% Plot Refined Uncertainty with Distinct Line Styles and Colors
figure('Position', [100, 100, 1000, 800]);
hold on;

% Define line styles and colors
lineStyles = {'-', '--', ':', '-.'}; % Different line styles
colors = lines(length(alphas)); % Generate distinct colors automatically

for i = 1:length(alphas)
    if alphas(i) == alpha_theoretical
        % Special styling for theoretical alpha
        plot(time, uncertainty_results(i, :), 'DisplayName', ...
            sprintf('Alpha = %.3f (Theoretical)', alphas(i)), 'LineWidth', 2, ...
            'LineStyle', '--', 'Color', 'k');  % Black dashed line
    else
        % Cycle through line styles for other alphas
        lineStyle = lineStyles{mod(i-1, length(lineStyles)) + 1};
        plot(time, uncertainty_results(i, :), 'DisplayName', ...
            sprintf('Alpha = %.2f', alphas(i)), 'LineWidth', 1.5, ...
            'LineStyle', lineStyle, 'Color', colors(i, :));
    end
end

xlabel('Time (s)', 'FontSize', 12);
ylabel('Uncertainty (rad)', 'FontSize', 12);
title('Refined Uncertainty in Fused Angle for Alphas (0.9 to 1.0)', 'FontSize', 14);
legend('FontSize', 10, 'Location', 'best');
grid on;
hold off;

% Print all the uncertainty values for refined alpha range
fprintf('Uncertainty values for refined alpha range (0.8 to 1.0):\n');
for i = 1:length(alphas)
    fprintf('Alpha = %.4f: %.7f\n', alphas(i), uncertainty_results(i, end));
end
