function [bearing, bearing_measurement_noise] = measure_bearing(unicycle_configuration, landmark_position, bearing_measurement_noise_covariance)
    assert(isequal(size(unicycle_configuration), [3, 1]));
    assert(isequal(size(landmark_position), [2, 1]));
    assert(isequal(size(bearing_measurement_noise_covariance), [1, 1]));
    bearing_measurement_noise = mvnrnd(zeros(1, 1), bearing_measurement_noise_covariance);
    bearing = wrapToPi(compute_bearing(unicycle_configuration, landmark_position) + bearing_measurement_noise);
end
