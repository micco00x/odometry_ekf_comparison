clc
clear
close all

% Simulation hparams:
simulation_duration = 25.0; % [s]
sampling_interval = 0.001; % [s]
iterations = simulation_duration / sampling_interval;
measurements_type = [MeasurementType.Bearing; MeasurementType.Distance; MeasurementType.Bearing; MeasurementType.Distance]; % Bearing or Distance

% Initial configuration of the unicycle:
unicycle_configuration = zeros(3, 1); % [m], [m], [rad]

% Commands to be applied to the unicycle
control_input = [1.0; 0.1]; % [m/s], [rad/s]

% Position of the landmark:
landmarks_position = [[7.0, 3.0]; [7.0, 3.0]; [19.0, -4.0]; [19.0, -4.0]];

% ASSERT: double check sizes.
num_measurements = size(measurements_type, 1);
assert(size(measurements_type, 1) == size(landmarks_position, 1));

% Odometric localization and EKF data:
unicycle_configuration_estimated_with_odometry = unicycle_configuration;
unicycle_configuration_estimated_with_ekf = unicycle_configuration;
unicycle_covariance_ekf = 1e-3 * eye(3, 3);
process_noise_covariance = diag([1e-6, 1e-6, 1e-7]);
bearing_measurement_noise_covariance = 1e-5;
distance_measurement_noise_covariance = 1e-5;
measurements_noise_covariance = zeros(num_measurements);
for k = 1:num_measurements
    if measurements_type(k) == MeasurementType.Bearing
        % MeasurementType.Bearing
        measurements_noise_covariance(k, k) = bearing_measurement_noise_covariance;
    else
        % MeasurementType.Distance
        measurements_noise_covariance(k, k) = distance_measurement_noise_covariance;
    end
end


% Variables to be used for plotting:
time = linspace(0.0, iterations * sampling_interval, iterations);
unicycle_configuration_log = zeros(iterations, 3);
unicycle_configuration_odometry_log = zeros(iterations, 3);
unicycle_configuration_ekf_log = zeros(iterations, 3);
process_noise_log = zeros(iterations, 3);
measurements_noise_log = zeros(iterations, num_measurements);
measurements_log = zeros(iterations, num_measurements);

% Run simulation:
for iter = 1:iterations
    % Simulation step:
    [unicycle_configuration, process_noise] = simulate_unicycle_motion(unicycle_configuration, control_input, sampling_interval, process_noise_covariance);
    [measurements, measurements_noise] = read_measurements(unicycle_configuration, landmarks_position, measurements_type, bearing_measurement_noise_covariance, distance_measurement_noise_covariance);

    % Odometric localization:
    unicycle_configuration_estimated_with_odometry = odometric_localization(unicycle_configuration_estimated_with_odometry, control_input, sampling_interval);

    % EKF localization:
    [unicycle_configuration_estimated_with_ekf, unicycle_covariance_ekf] = EKF(unicycle_configuration_estimated_with_ekf, unicycle_covariance_ekf, control_input, sampling_interval, process_noise_covariance, landmarks_position, measurements, measurements_noise_covariance, measurements_type);

    % Log:
    unicycle_configuration_log(iter, :) = unicycle_configuration;
    unicycle_configuration_odometry_log(iter, :) = unicycle_configuration_estimated_with_odometry;
    unicycle_configuration_ekf_log(iter, :) = unicycle_configuration_estimated_with_ekf;
    process_noise_log(iter, :) = process_noise;
    measurements_noise_log(iter, :) = measurements_noise;
    measurements_log(iter, :) = measurements;
end

% Plots:
% Odometry plot:
figure
plot_configuration_comparison(time, unicycle_configuration_log, unicycle_configuration_odometry_log, 'ground truth', 'odometric localization');

% EKF plot:
figure
plot_configuration_comparison(time, unicycle_configuration_log, unicycle_configuration_ekf_log, 'ground truth', 'EKF');

% Unicycle plot:
num_unicycles_to_draw = 10;
figure
scatter(landmarks_position(:, 1), landmarks_position(:, 2));
hold on
draw_unicycle_from_trajectory(unicycle_configuration_log, num_unicycles_to_draw, 'black');
hold on
draw_unicycle_from_trajectory(unicycle_configuration_odometry_log, num_unicycles_to_draw, 'red');
hold on
draw_unicycle_from_trajectory(unicycle_configuration_ekf_log, num_unicycles_to_draw, 'green');
legend('landmark', 'ground truth', 'odometric localization', 'EKF', 'Location', 'northwest');

% Process noise plot:
figure
subplot(3, 1, 1);
plot(time, process_noise_log(:, 1));
title('process noise');
ylabel('x [m]');
subplot(3, 1, 2);
plot(time, process_noise_log(:, 2));
ylabel('y [m]');
subplot(3, 1, 3);
plot(time, process_noise_log(:, 3));
xlabel('[s]');
ylabel('\theta [rad]');
