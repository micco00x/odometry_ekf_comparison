function [measurements, measurements_noise] = read_measurements(unicycle_configuration, landmarks, measurements_info, bearing_measurement_noise_covariance, distance_measurement_noise_covariance)
    num_measurements = size(measurements_info, 1);
    measurements = zeros(num_measurements, 1);
    measurements_noise = zeros(num_measurements, 1);
    for k = 1:num_measurements
        landmark_id = measurements_info(k).landmark_id;
        landmark_position = landmarks(landmark_id, :)';
        if measurements_info(k).type == MeasurementType.Bearing
            % MeasurementType.Bearing
            [measurements(k), measurements_noise(k)] = measure_bearing(unicycle_configuration, landmark_position, bearing_measurement_noise_covariance);
        else
            % MeasurementType.Distance
            [measurements(k), measurements_noise(k)] = measure_distance(unicycle_configuration, landmark_position, distance_measurement_noise_covariance);
        end
    end
end

