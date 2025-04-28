% Plot noise

clear; clc; close all;

% Define the path to the data file
dataFilePath = "../../data/static/static_table_log_raw_4g_500dps.csv";

% Check if the file exists
if ~isfile(dataFilePath)
    error('Data file does not exist: %s', dataFilePath);
end

% Read data from log file into a table
data = readtable(dataFilePath);

% Identify relevant data
timeIdx = contains(data.Properties.VariableNames, 'Time');
time = data{:, timeIdx};

accelIdx = contains(data.Properties.VariableNames, 'Accel');
accelTable = data(:, accelIdx);

gyroIdx = contains(data.Properties.VariableNames, 'Gyro');
gyroTable = data(:, gyroIdx);

% Isolate the noise
accelNoiseTable = accelTable - mean(accelTable);
gyroNoiseTable = gyroTable - mean(gyroTable);

% Sampling information
dt = 0.008; % seconds per sample
fs = 1/dt;

% Default figure size
figure_size = [100, 100, 1000, 800];

% Analyze accelerometer noise
for table={accelNoiseTable}

    % Plot noise
    figure(Position=figure_size);
    sgtitle('Accelerometer Noise');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        X_name = entry.Properties.VariableNames(1);

        plot(time, entry{:, :}, 'LineWidth', 1);
        title(strcat(X_name,' Noise'));
        xlabel('Time (ms)');
        xlim([time(1), time(end)]);
        ylabel('Linear Acceleration (m/s^2)');
        grid on;

        noise_mean = mean(entry{:,:});
        noise_std = std(entry{:,:});

        yline(noise_mean, 'Color', 'r', 'LineWidth', 1.5);
        yline(noise_mean + noise_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
        yline(noise_mean - noise_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);

        text(time(end), noise_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), noise_mean + noise_std, ' +\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), noise_mean - noise_std, ' -\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    end

    % Plot noise distribution
    figure(Position=figure_size);
    sgtitle('Accelerometer Noise Amplitude Distribution');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        noise_mean = mean(entry{:,:});
        noise_std = std(entry{:,:});

        x = linspace(min(entry{:, :}), max(entry{:, :}), 100);
        y = normpdf(x, noise_mean, noise_std);

        X_name = entry.Properties.VariableNames(1);

        hold on;
        histogram(entry{:, :}, 30, 'Normalization', 'pdf');
        plot(x, y);
        title(strcat(X_name,' Amplitude Distribution of Noise'));
        xlabel('Amplitude');
        ylabel('Probability Density');
    end

    % Plot autocorrelation
    figure(Position=figure_size);
    sgtitle('Accelerometer Noise Autocorrelation (Unbiased Estimate)');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Compute autocorrelation
        [r, lags] = xcorr(entry{:, :}, 'unbiased');
        taus = lags * dt;   % Convert from samples to seconds

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        X_name = entry.Properties.VariableNames(1);

        plot(taus, r, 'LineWidth', 1);
        title(strcat(X_name,' Autocorrelation'));
        xlabel('\tau (s)');
        xlim([-40, 40]);
        ylabel('Linear Acceleration Squared (m/s^2)^2');
        grid on;
    end

    % Plot power spectral density
    figure(Position=figure_size);
    sgtitle('Accelerometer Noise Power Spectral Density (Periodogram Estimate)');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Compute power spectral density
        values = entry{:, :};
        [pxx, f] = periodogram(values, rectwin(length(values)), length(values), fs, 'psd');

        % Remove DC component
        f = f(2:end);
        db_power = pow2db(pxx(2:end));
        db_power_mean = mean(db_power);

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        X_name = entry.Properties.VariableNames(1);

        plot(f, db_power, 'LineWidth', 1);
        yline(db_power_mean, 'Color', 'r', 'LineWidth', 1.5);
        text(f(end), db_power_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        title(strcat(X_name,' Power Spectral Density'));
        xlabel('Frequency (Hz)');
        xlim([f(1), f(end)]);
        ylabel('Power/frequency (db/Hz)');
        grid on;
    end

    % Plot rolling mean and variance
    window_length = floor(0.5 / dt);   % 0.5 second window length, in unit samples

    figure(Position=figure_size);
    sgtitle(sprintf('Accelerometer Noise Rolling Mean and Variance (Window Length: %d ms)', window_length * dt * 1000));
    hold on;
    subplotcount = 1;

    for entry = table{1}
        % Extract values
        values = entry{:, :};
        X_name = string(entry.Properties.VariableNames(1));

        % Compute rolling mean and variance
        rolling_mean = movmean(values, window_length);
        rolling_var = movvar(values, window_length);

        % Compute rolling mean and variance standard deviation
        rolling_mean_mean = mean(rolling_mean);
        rolling_mean_std = std(rolling_mean);

        rolling_var_mean = mean(rolling_var);
        rolling_var_std = std(rolling_var);

        % Display rolling mean and variance standard deviation
        fprintf('%s rolling mean standard deviation: %f\n', X_name, rolling_mean_std);
        fprintf('%s rolling variance standard deviation: %f\n', X_name, rolling_var_std);

        % Plot rolling mean
        subplot(width(table{1}), 2, subplotcount);
        subplotcount = subplotcount + 1;

        plot(time, rolling_mean, 'LineWidth', 1.5);
        hold on;
        yline(rolling_mean_mean, 'Color', 'r', 'LineWidth', 1.5);
        yline(rolling_mean_mean + rolling_mean_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
        yline(rolling_mean_mean - rolling_mean_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
        hold off;
        title(sprintf('%s Noise Rolling Mean', X_name));
        xlabel('Time (ms)');
        ylabel('Mean Linear Acceleration (m/s^2)');
        xlim([time(1), time(end)]);

        text(time(end), rolling_mean_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_mean_mean + rolling_mean_std, ' +\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_mean_mean - rolling_mean_std, ' -\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

        % Plot rolling variance
        subplot(width(table{1}), 2, subplotcount);
        subplotcount = subplotcount + 1;

        plot(time, rolling_var, 'LineWidth', 1.5);
        hold on;
        yline(rolling_var_mean, 'Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Mean');
        yline(rolling_var_mean + rolling_var_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', '+\sigma');
        yline(rolling_var_mean - rolling_var_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', '-\sigma');
        hold off;
        title(sprintf('%s Rolling Variance', X_name));
        xlabel('Time (ms)');
        ylabel('Variance Linear Acceleration (m/s^2)^2');
        xlim([time(1), time(end)]);

        text(time(end), rolling_var_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_var_mean + rolling_var_std, ' +\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_var_mean - rolling_var_std, ' -\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    end
end

% Analyze gyroscope noise
for table={gyroNoiseTable}

    % Plot noise
    figure(Position=figure_size);
    sgtitle('Gyroscope Noise');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        X_name = entry.Properties.VariableNames(1);

        plot(time, entry{:, :}, 'LineWidth', 1);
        title(strcat(X_name,' Noise'));
        xlabel('Time (ms)');
        xlim([time(1), time(end)]);
        ylabel('Angular Velocity (rad/s)');
        grid on;

        noise_mean = mean(entry{:,:});
        noise_std = std(entry{:,:});

        yline(noise_mean, 'Color', 'r', 'LineWidth', 1.5);
        yline(noise_mean + noise_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
        yline(noise_mean - noise_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);

        text(time(end), noise_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), noise_mean + noise_std, ' +\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), noise_mean - noise_std, ' -\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    end

    % Plot noise distribution
    figure(Position=figure_size);
    sgtitle('Gyroscope Noise Amplitude Distribution');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        noise_mean = mean(entry{:,:});
        noise_std = std(entry{:,:});

        x = linspace(min(entry{:, :}), max(entry{:, :}), 100);
        y = normpdf(x, noise_mean, noise_std);

        X_name = entry.Properties.VariableNames(1);

        hold on;
        histogram(entry{:, :}, 30, 'Normalization', 'pdf');
        plot(x, y);
        title(strcat(X_name,' Amplitude Distribution of Noise'));
        xlabel('Amplitude');
        ylabel('Probability Density');
    end

    % Plot autocorrelation
    figure(Position=figure_size);
    sgtitle('Gyroscope Noise Autocorrelation (Unbiased Estimate)');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Compute autocorrelation
        [r, lags] = xcorr(entry{:, :}, 'unbiased');
        taus = lags * dt;   % Convert from samples to seconds

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        X_name = entry.Properties.VariableNames(1);

        plot(taus, r, 'LineWidth', 1);
        title(strcat(X_name,' Autocorrelation'));
        xlabel('\tau (s)');
        xlim([-20, 20]);
        ylabel('Angular Velocity Squared (rad/s)^2');
        grid on;
    end

    % Plot power spectral density
    figure(Position=figure_size);
    sgtitle('Gyroscope Noise Power Spectral Density (Periodogram Estimate)');
    hold on;
    subplotcount = 1;

    for entry=table{1}

        % Compute power spectral density
        values = entry{:, :};
        [pxx, f] = periodogram(values, rectwin(length(values)), length(values), fs, 'psd');

        % Remove DC component
        f = f(2:end);
        db_power = pow2db(pxx(2:end));
        db_power_mean = mean(db_power);

        % Plot on subplot
        subplot(width(table{1}), 1, subplotcount);
        subplotcount = subplotcount + 1;

        X_name = entry.Properties.VariableNames(1);

        plot(f, db_power, 'LineWidth', 1);
        yline(db_power_mean, 'Color', 'r', 'LineWidth', 1.5);
        text(f(end), db_power_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        title(strcat(X_name,' Power Spectral Density'));
        xlabel('Frequency (Hz)');
        xlim([f(1), f(end)]);
        ylabel('Power/frequency (db/Hz)');
        grid on;
    end

    % Plot rolling mean and variance
    window_length = floor(0.5 / dt);   % 0.5 second window length, in unit samples

    figure(Position=figure_size);
    sgtitle(sprintf('Gyroscope Noise Rolling Mean and Variance (Window Length: %d ms)', window_length * dt * 1000));
    hold on;
    subplotcount = 1;

    for entry = table{1}
        % Extract values
        values = entry{:, :};
        X_name = string(entry.Properties.VariableNames(1));

        % Compute rolling mean and variance
        rolling_mean = movmean(values, window_length);
        rolling_var = movvar(values, window_length);

        % Compute rolling mean and variance standard deviation
        rolling_mean_mean = mean(rolling_mean);
        rolling_mean_std = std(rolling_mean);

        rolling_var_mean = mean(rolling_var);
        rolling_var_std = std(rolling_var);

        % Display rolling mean and variance standard deviation
        fprintf('%s rolling mean standard deviation: %f\n', X_name, rolling_mean_std);
        fprintf('%s rolling variance standard deviation: %f\n', X_name, rolling_var_std);

        % Plot rolling mean
        subplot(width(table{1}), 2, subplotcount);
        subplotcount = subplotcount + 1;

        plot(time, rolling_mean, 'LineWidth', 1.5);
        hold on;
        yline(rolling_mean_mean, 'Color', 'r', 'LineWidth', 1.5);
        yline(rolling_mean_mean + rolling_mean_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
        yline(rolling_mean_mean - rolling_mean_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2);
        hold off;
        title(sprintf('%s Noise Rolling Mean', X_name));
        xlabel('Time (ms)');
        ylabel('Mean Angular Velocity (rad/s)');
        xlim([time(1), time(end)]);

        text(time(end), rolling_mean_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_mean_mean + rolling_mean_std, ' +\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_mean_mean - rolling_mean_std, ' -\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

        % Plot rolling variance
        subplot(width(table{1}), 2, subplotcount);
        subplotcount = subplotcount + 1;

        plot(time, rolling_var, 'LineWidth', 1.5);
        hold on;
        yline(rolling_var_mean, 'Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Mean');
        yline(rolling_var_mean + rolling_var_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', '+\sigma');
        yline(rolling_var_mean - rolling_var_std, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', '-\sigma');
        hold off;
        title(sprintf('%s Rolling Variance', X_name));
        xlabel('Time (ms)');
        ylabel('Variance Angular Velocity (rad/s)^2');
        xlim([time(1), time(end)]);

        text(time(end), rolling_var_mean, ' Mean', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_var_mean + rolling_var_std, ' +\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        text(time(end), rolling_var_mean - rolling_var_std, ' -\sigma', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    end
end
