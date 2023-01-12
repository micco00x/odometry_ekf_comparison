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
control_input = [1.0; 0.0]; % [m/s], [rad/s]

% Position of the landmark:
landmarks_position = [[7.0, 3.0]; [7.0, 3.0]; [19.0, -4.0]; [19.0, -4.0]];

% ASSERT: double check sizes.
num_measurements = size(measurements_type, 1);
assert(size(measurements_type, 1) == size(landmarks_position, 1));

% Odometric localization and EKF data:
unicycle_configuration_estimated_with_odometry = unicycle_configuration;
unicycle_configuration_estimated_with_ekf = unicycle_configuration;
unicycle_covariance_ekf = 0.1 * eye(3, 3);
process_noise_covariance = diag([1e-5, 1e-5, 1e-7]);
measurements_noise_covariance = zeros(num_measurements);

% Variables to be used for plotting:
time = linspace(0.0, iterations * sampling_interval, iterations);

x = zeros(iterations, 1);
y = zeros(iterations, 1);
theta = zeros(iterations, 1);

x_odom = zeros(iterations, 1);
y_odom = zeros(iterations, 1);
theta_odom = zeros(iterations, 1);

x_ekf = zeros(iterations, 1);
y_ekf = zeros(iterations, 1);
theta_ekf = zeros(iterations, 1);

x_noise = zeros(iterations, 1);
y_noise = zeros(iterations, 1);
theta_noise = zeros(iterations, 1);

% Run simulation:
for iter = 1:iterations
    % Simulation step:
    [unicycle_configuration, process_noise] = simulate_unicycle_motion(unicycle_configuration, control_input, sampling_interval, process_noise_covariance);

    % Odometric localization:
    unicycle_configuration_estimated_with_odometry = odometric_localization(unicycle_configuration_estimated_with_odometry, control_input, sampling_interval);

    % EKF localization:
    measurements = zeros(num_measurements, 1);
    for k = 1:num_measurements
        if measurements_type(k) == MeasurementType.Bearing
            % MeasurementType.Bearing
            measurements(k) = measure_bearing(unicycle_configuration, landmarks_position(k, :));
        else
            % MeasurementType.Distance
            measurements(k) = measure_distance(unicycle_configuration, landmarks_position(k, :));
        end
    end
    unicycle_configuration_estimated_with_ekf = EKF(unicycle_configuration_estimated_with_ekf, unicycle_covariance_ekf, control_input, sampling_interval, process_noise_covariance, landmarks_position, measurements, measurements_noise_covariance, measurements_type);

    % Log:
    x(iter) = unicycle_configuration(1);
    y(iter) = unicycle_configuration(2);
    theta(iter) = unicycle_configuration(3);

    x_odom(iter) = unicycle_configuration_estimated_with_odometry(1);
    y_odom(iter) = unicycle_configuration_estimated_with_odometry(2);
    theta_odom(iter) = unicycle_configuration_estimated_with_odometry(3);

    x_ekf(iter) = unicycle_configuration_estimated_with_ekf(1);
    y_ekf(iter) = unicycle_configuration_estimated_with_ekf(2);
    theta_ekf(iter) = unicycle_configuration_estimated_with_ekf(3);
    
    x_noise(iter) = process_noise(1);
    y_noise(iter) = process_noise(2);
    theta_noise(iter) = process_noise(3);
end

% Plots:
% Odometry plot:
figure
plot_configuration_comparison(time, x, y, theta, x_odom, y_odom, theta_odom, 'ground truth', 'odometric localization');

% EKF plot:
figure
plot_configuration_comparison(time, x, y, theta, x_ekf, y_ekf, theta_ekf, 'ground truth', 'EKF');

% Unicycle plot:
num_unicycles_to_draw = 10;
figure
scatter(landmarks_position(:, 1), landmarks_position(:, 2));
hold on
draw_unicycle_from_trajectory(x, y, theta, num_unicycles_to_draw, 'black');
hold on
draw_unicycle_from_trajectory(x_odom, y_odom, theta_odom, num_unicycles_to_draw, 'red');
hold on
draw_unicycle_from_trajectory(x_ekf, y_ekf, theta_ekf, num_unicycles_to_draw, 'green');
legend('landmark', 'ground truth', 'odometric localization', 'EKF', 'Location', 'northwest');

% Process noise plot:
figure
subplot(3, 1, 1);
plot(time, x_noise);
title('process noise');
ylabel('x [m]');
subplot(3, 1, 2);
plot(time, y_noise);
ylabel('y [m]');
subplot(3, 1, 3);
plot(time, theta_noise);
xlabel('[s]');
ylabel('\theta [rad]');
