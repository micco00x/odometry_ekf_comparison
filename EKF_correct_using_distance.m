function [unicycle_configuration_corrected, unicycle_covariance_corrected] = EKF_correct_using_distance(unicycle_configuration_predicted, unicycle_covariance_predicted, landmark_position, distance_measurement, measurement_noise_covariance)
    % Linearization of the measurement model:
    x = unicycle_configuration_predicted(1);
    y = unicycle_configuration_predicted(2);
    x_l = landmark_position(1);
    y_l = landmark_position(2);
    H = [(x - x_l) / sqrt((x - x_l) ^ 2 + (y - y_l) ^ 2), ...
         (y - y_l) / sqrt((x - x_l) ^ 2 + (y - y_l) ^ 2), ...
         0];
    % Correct computing innovation and kalman gain matrix:
    unicycle_position_predicted = unicycle_configuration_predicted(1:2);
    innovation = distance_measurement - norm(unicycle_position_predicted - landmark_position);
    kalman_gain_matrix = unicycle_covariance_predicted * H' * inv(H * unicycle_covariance_predicted * H' + measurement_noise_covariance);
    unicycle_configuration_corrected = unicycle_configuration_predicted + kalman_gain_matrix * innovation;
    unicycle_configuration_corrected(3) = wrap_angle(unicycle_configuration_corrected(3));
    unicycle_covariance_corrected = unicycle_covariance_predicted - kalman_gain_matrix * H * unicycle_covariance_predicted;
end
