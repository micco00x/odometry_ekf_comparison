function [unicycle_configuration_corrected, unicycle_covariance_corrected] = EKF_correct_using_bearing(unicycle_configuration_predicted, unicycle_covariance_predicted, landmark_position, bearing_measurement, measurement_noise_covariance)
    % Linearization of the measurement model:
    x = unicycle_configuration_predicted(1);
    y = unicycle_configuration_predicted(2);
    x_l = landmark_position(1);
    y_l = landmark_position(2);
    H = [ (y_l - y) / ((x_l - x) ^ 2 + (y_l - y) ^ 2), ...
         -(x_l - x) / ((x_l - x) ^ 2 + (y_l - y) ^ 2), ...
         -1];
    % Correct computing innovation and kalman gain matrix:
    innovation = bearing_measurement - compute_bearing(unicycle_configuration_predicted, landmark_position);
    kalman_gain_matrix = unicycle_covariance_predicted * H' * inv(H * unicycle_covariance_predicted * H' + measurement_noise_covariance);
    unicycle_configuration_corrected = unicycle_configuration_predicted + kalman_gain_matrix * innovation;
    unicycle_configuration_corrected(3) = wrap_angle(unicycle_configuration_corrected(3));
    unicycle_covariance_corrected = unicycle_covariance_predicted - kalman_gain_matrix * H * unicycle_covariance_predicted;
end
