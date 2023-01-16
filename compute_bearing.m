function bearing = compute_bearing(unicycle_configuration, landmark_position)
    unicycle_position = unicycle_configuration(1:2);
    unicycle_orientation = unicycle_configuration(3);
    bearing = wrap_angle(atan2(landmark_position(2) - unicycle_position(2), landmark_position(1) - unicycle_position(1)) - unicycle_orientation);
end