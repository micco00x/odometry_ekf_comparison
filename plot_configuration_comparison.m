function plot_configuration_comparison(time, unicycle_configurations_ref, unicycle_configurations_est, legend_ref_name, legend_est_name)
    x_ref = unicycle_configurations_ref(:, 1);
    y_ref = unicycle_configurations_ref(:, 2);
    theta_ref = unicycle_configurations_ref(:, 3);
    x_est = unicycle_configurations_est(:, 1);
    y_est = unicycle_configurations_est(:, 2);
    theta_est = unicycle_configurations_est(:, 3);
    subplot(3, 1, 1);
    plot(time, x_ref, time, x_est);
    title(legend_est_name);
    legend(legend_ref_name, legend_est_name, 'Location', 'northwest');
    ylabel('x [m]');
    subplot(3, 1, 2);
    plot(time, y_ref, time, y_est);
    ylabel('y [m]');
    subplot(3, 1, 3);
    plot(time, theta_ref, time, theta_est);
    xlabel('[s]');
    ylabel('\theta [rad]');
end

