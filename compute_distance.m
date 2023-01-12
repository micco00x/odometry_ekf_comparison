function distance = compute_distance(unicycle_configuration, landmark_position)
    unicycle_position = unicycle_configuration(1:2);
    distance = norm(unicycle_position - landmark_position);
end
