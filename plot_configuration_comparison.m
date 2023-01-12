function plot_configuration_comparison(time, x_ref, y_ref, theta_ref, x_est, y_est, theta_est, legend_ref_name, legend_est_name)
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

