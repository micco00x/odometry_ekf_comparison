function [distance, distance_measurement_noise] = measure_distance(unicycle_configuration, landmark_position, distance_measurement_noise_covariance)
    distance_measurement_noise = mvnrnd(zeros(1, 1), distance_measurement_noise_covariance);
    distance = compute_distance(unicycle_configuration, landmark_position) + distance_measurement_noise;
end
