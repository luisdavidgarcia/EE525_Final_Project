% Temporary Angular Velocityr Complimentary Filter


%% Precompute Accelerometer-Based Angles
theta_accel = atan2(aY, sqrt(aX.^2 + aZ.^2));

% Angular velocity derived from accelerometer angles
theta_dot_accel = diff(theta_accel) / dt; 

% Noise variance for dynamic data (angular velocity)
varAccelVel = var(theta_dot_accel); 
varGyroVel = var(gX); 

% Theoretical beta (velocity fusion)
beta = varAccelVel / (varAccelVel + varGyroVel);
fprintf('Theoretical beta (velocity fusion): %.3f\n', beta);

%% Initialize Angular Velocity
theta_dot = zeros(1, n-1); % Fused angular velocity

%% Loop through dynamic data
for k = 2:n-1
    % Gyroscope angular velocity (direct measurement)
    theta_dot_gyro = gX(k);

    % Complementary filter for angular velocity
    theta_dot(k-1) = beta * theta_dot_gyro + (1 - beta) * theta_dot_accel(k-1); % Match indexing
end

%% Plot Results for Angular Velocity
figure;
% Plot fused angular velocity
plot(time(1:end-1), theta_dot, 'DisplayName', 'Fused Angular Velocity (Complementary Filter)');
hold on;

% Plot gyroscope-only angular velocity
plot(time(1:end-1), gX(1:end-1), '--', 'DisplayName', 'Gyroscope-Only Angular Velocity');

% Plot accelerometer-derived angular velocity
plot(time(1:end-1), theta_dot_accel, ':', 'DisplayName', 'Accelerometer-Derived Angular Velocity');

% Add legend and labels
legend;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Comparison of Fused Angular Velocity vs. Individual Sources');
grid on;




figure;
plot(time(1:end-1), sqrt(var_theta_dot), 'DisplayName', 'Uncertainty in Fused Angular Velocity (std dev)');
xlabel('Time (s)');
ylabel('Uncertainty (rad/s)');
title('Uncertainty in Fused Angular Velocity Over Time (Complementary Filter)');
grid on;
legend;