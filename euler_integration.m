function integrated_unicycle_configuration = euler_integration(unicycle_configuration, control_input, sampling_interval)
    driving_velocity = control_input(1);
    steering_velocity = control_input(2);
    x = unicycle_configuration(1) + driving_velocity * sampling_interval * cos(unicycle_configuration(3));
    y = unicycle_configuration(2) + driving_velocity * sampling_interval * sin(unicycle_configuration(3));
    theta = wrapToPi(unicycle_configuration(3) + steering_velocity * sampling_interval);
    integrated_unicycle_configuration = [x; y; theta];
end