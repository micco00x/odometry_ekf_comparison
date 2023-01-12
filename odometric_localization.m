function unicycle_configuration_estimated = odometric_localization(unicycle_configuration_estimated, control_input, sampling_interval)
    unicycle_configuration_estimated = euler_integration(unicycle_configuration_estimated, control_input, sampling_interval);
end
