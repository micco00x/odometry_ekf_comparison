function distance = compute_distance(unicycle_configuration, landmark_position)
    assert(isequal(size(unicycle_configuration), [3, 1]));
    assert(isequal(size(landmark_position), [2, 1]));
    unicycle_position = unicycle_configuration(1:2);
    distance = norm(unicycle_position - landmark_position);
end
