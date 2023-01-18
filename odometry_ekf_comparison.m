% Clean workspace:
clc
clear
close all

% Simulation hparams:
rng(1) % use a seed (non-negative number of 'default')
simulation_duration = 25.0; % [s]
sampling_interval = 0.1; % [s]
num_unicycles_to_draw = 20;

unicycle_covariance_ekf = 1e-3 * eye(3, 3);
process_noise_covariance = diag([1e-5, 1e-5, 1e-4]);
bearing_measurement_noise_covariance = 1e-5;
distance_measurement_noise_covariance = 1e-5;

iterations = simulation_duration / sampling_interval;

% Initial configuration of the unicycle:
unicycle_configuration = zeros(3, 1); % [m], [m], [rad]

% Commands to be applied to the unicycle:
control_input = [2.0; 0.2]; % [m/s], [rad/s]

% Position of the landmarks:
landmarks = [[4.0, 5.0]; [-8.0, 20.0]];

% Measurements type and relative landmarks:
measurements_info = [
    MeasurementInfo(1, MeasurementType.Bearing); ...
    MeasurementInfo(1, MeasurementType.Distance); ...
    %MeasurementInfo(2, MeasurementType.Bearing); ...
    %MeasurementInfo(2, MeasurementType.Distance)
];
num_measurements = size(measurements_info, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% DO NOT MODIFY ANYTHING BELOW UNLESS YOU KNOW WHAT YOU ARE DOING %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Odometric localization and EKF data:
unicycle_configuration_estimated_with_odometry = unicycle_configuration;
unicycle_configuration_estimated_with_ekf = unicycle_configuration;
measurements_noise_covariance = zeros(num_measurements);
for k = 1:num_measurements
    if measurements_info(k).type == MeasurementType.Bearing
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
process_noise = zeros(3, 1);
for iter = 1:iterations
    % Read measurements from sensors:
    [measurements, measurements_noise] = read_measurements(unicycle_configuration, landmarks, measurements_info, bearing_measurement_noise_covariance, distance_measurement_noise_covariance);

    % Odometric localization:
    unicycle_configuration_estimated_with_odometry = odometric_localization(unicycle_configuration_estimated_with_odometry, control_input, sampling_interval);

    % EKF localization:
    [unicycle_configuration_estimated_with_ekf, unicycle_covariance_ekf] = EKF(unicycle_configuration_estimated_with_ekf, unicycle_covariance_ekf, control_input, sampling_interval, process_noise_covariance, landmarks, measurements, measurements_noise_covariance, measurements_info);

    % Log:
    unicycle_configuration_log(iter, :) = unicycle_configuration;
    unicycle_configuration_odometry_log(iter, :) = unicycle_configuration_estimated_with_odometry;
    unicycle_configuration_ekf_log(iter, :) = unicycle_configuration_estimated_with_ekf;
    process_noise_log(iter, :) = process_noise;
    measurements_noise_log(iter, :) = measurements_noise;
    measurements_log(iter, :) = measurements;

    % Simulation step:
    [unicycle_configuration, process_noise] = simulate_unicycle_motion(unicycle_configuration, control_input, sampling_interval, process_noise_covariance);
end

% Plots:
% Odometry plot:
figure
plot_configuration_comparison(time, unicycle_configuration_log, unicycle_configuration_odometry_log, 'ground truth', 'odometric localization');

% EKF plot:
figure
plot_configuration_comparison(time, unicycle_configuration_log, unicycle_configuration_ekf_log, 'ground truth', 'EKF');

% Unicycle plot:
figure
measured_landmarks_id = zeros(num_measurements, 1);
for k = 1:num_measurements
    measured_landmarks_id(k) = measurements_info(k).landmark_id;
end
scatter(landmarks(measured_landmarks_id, 1), landmarks(measured_landmarks_id, 2));
hold on
draw_unicycle_from_trajectory(unicycle_configuration_log, num_unicycles_to_draw, 'black');
hold on
draw_unicycle_from_trajectory(unicycle_configuration_odometry_log, num_unicycles_to_draw, 'red');
hold on
draw_unicycle_from_trajectory(unicycle_configuration_ekf_log, num_unicycles_to_draw, 'green');
legend('landmark', 'ground truth', 'odometric localization', 'EKF', 'Location', 'northeast');

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
