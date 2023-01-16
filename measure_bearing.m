function [bearing, bearing_measurement_noise] = measure_bearing(unicycle_configuration, landmark_position, bearing_measurement_noise_covariance)
    bearing_measurement_noise = mvnrnd(zeros(1, 1), bearing_measurement_noise_covariance);
    bearing = wrap_angle(compute_bearing(unicycle_configuration, landmark_position) + bearing_measurement_noise);
end
