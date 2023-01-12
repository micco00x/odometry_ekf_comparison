function [unicycle_configuration_predicted, unicycle_covariance_predicted] = EKF_predict(unicycle_configuration_estimated, unicycle_covariance_estimated, control_input, sampling_interval, process_noise_covariance)
    unicycle_orientation_estimated = unicycle_configuration_estimated(2);
    driving_velocity = control_input(1);
    % Linearization of the kinematic model of the unicycle:
    F = [1, 0, -driving_velocity * sin(unicycle_orientation_estimated); ...
         0, 1,  driving_velocity * cos(unicycle_orientation_estimated); ...
         0, 0, 1];
    % Euler integration and covariance matrix:
    unicycle_configuration_predicted = euler_integration(unicycle_configuration_estimated, control_input, sampling_interval);
    unicycle_covariance_predicted = F * unicycle_covariance_estimated * F' + process_noise_covariance;
end
