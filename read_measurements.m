function [measurements, measurements_noise] = read_measurements(unicycle_configuration, landmarks_position, measurements_type, measurements_noise_covariance)
    num_measurements = size(measurements_type, 1);
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
    measurements_noise = mvnrnd(zeros(num_measurements, 1), measurements_noise_covariance);
    measurements = measurements + measurements_noise;
    for k = 1:num_measurements
        if measurements_type(k) == MeasurementType.Bearing
            measurements(k) = wrap_angle(measurements(k));
        end
    end
end

