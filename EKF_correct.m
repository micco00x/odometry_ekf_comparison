function [unicycle_configuration_corrected, unicycle_covariance_corrected] = EKF_correct(unicycle_configuration_predicted, unicycle_covariance_predicted, landmarks, measurements, measurements_noise_covariance, measurements_info)
    x = unicycle_configuration_predicted(1);
    y = unicycle_configuration_predicted(2); 
    unicycle_position_predicted = [x; y];
    % Linearization of the measurement model:
    num_measurements = size(measurements, 1);
    innovation = zeros(num_measurements, 1);
    H = zeros(num_measurements, 3);
    for k = 1:num_measurements
        landmark_id = measurements_info(k).landmark_id;
        landmark_position = landmarks(landmark_id, :)';
        x_l = landmark_position(1);
        y_l = landmark_position(2);
        if measurements_info(k).type == MeasurementType.Bearing
            % MeasurementType.Bearing
            innovation(k) = wrapToPi(measurements(k) - compute_bearing(unicycle_configuration_predicted, landmark_position));
            H(k, :) = [ (y_l - y) / ((x_l - x) ^ 2 + (y_l - y) ^ 2), ...
                       -(x_l - x) / ((x_l - x) ^ 2 + (y_l - y) ^ 2), ...
                       -1];
        else
            % MeasurementType.Distance
            innovation(k) = measurements(k) - norm(unicycle_position_predicted - landmark_position);
            H(k, :) = [(x - x_l) / sqrt((x - x_l) ^ 2 + (y - y_l) ^ 2), ...
                       (y - y_l) / sqrt((x - x_l) ^ 2 + (y - y_l) ^ 2), ...
                       0];
        end
    end
    % Correct computing innovation and kalman gain matrix:
    kalman_gain_matrix = unicycle_covariance_predicted * H' * inv(H * unicycle_covariance_predicted * H' + measurements_noise_covariance);
    unicycle_configuration_corrected = unicycle_configuration_predicted + kalman_gain_matrix * innovation;
    unicycle_configuration_corrected(3) = wrapToPi(unicycle_configuration_corrected(3));
    unicycle_covariance_corrected = unicycle_covariance_predicted - kalman_gain_matrix * H * unicycle_covariance_predicted;

end

