function [measurements, measurements_noise] = read_measurements(unicycle_configuration, landmarks_position, measurements_type, bearing_measurement_noise_covariance, distance_measurement_noise_covariance)
    num_measurements = size(measurements_type, 1);
    measurements = zeros(num_measurements, 1);
    measurements_noise = zeros(num_measurements, 1);
    for k = 1:num_measurements
        if measurements_type(k) == MeasurementType.Bearing
            % MeasurementType.Bearing
            [measurements(k), measurements_noise(k)] = measure_bearing(unicycle_configuration, landmarks_position(k, :)', bearing_measurement_noise_covariance);
        else
            % MeasurementType.Distance
            [measurements(k), measurements_noise(k)] = measure_distance(unicycle_configuration, landmarks_position(k, :)', distance_measurement_noise_covariance);
        end
    end
end

