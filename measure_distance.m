function [distance, distance_measurement_noise] = measure_distance(unicycle_configuration, landmark_position, distance_measurement_noise_covariance)
    assert(isequal(size(unicycle_configuration), [3, 1]));
    assert(isequal(size(landmark_position), [2, 1]));
    assert(isequal(size(distance_measurement_noise_covariance), [1, 1]));
    distance_measurement_noise = mvnrnd(zeros(1, 1), distance_measurement_noise_covariance);
    distance = compute_distance(unicycle_configuration, landmark_position) + distance_measurement_noise;
end
