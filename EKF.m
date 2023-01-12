function [unicycle_configuration_corrected, unicycle_covariance_corrected] = EKF(unicycle_configuration_estimated, unicycle_covariance_estimated, control_input, sampling_interval, process_noise_covariance, landmark_position, measurement, measurement_noise_covariance, measurement_type)
    [unicycle_configuration_predicted, unicycle_covariance_predicted] = EKF_predict(unicycle_configuration_estimated, unicycle_covariance_estimated, control_input, sampling_interval, process_noise_covariance);
    if strcmp(measurement_type, 'BearingMeasurement') == 1
        [unicycle_configuration_corrected, unicycle_covariance_corrected] = EKF_correct_using_bearing(unicycle_configuration_predicted, unicycle_covariance_predicted, landmark_position, measurement, measurement_noise_covariance);
    else % 'DistanceMeasurement'
        [unicycle_configuration_corrected, unicycle_covariance_corrected] = EKF_correct_using_distance(unicycle_configuration_predicted, unicycle_covariance_predicted, landmark_position, measurement, measurement_noise_covariance);
    end
end