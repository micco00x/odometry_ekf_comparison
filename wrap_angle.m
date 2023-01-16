% wrap_angle(theta) Wraps an angle theta to [-pi, pi].
function theta = wrap_angle(theta)
    theta = wrapToPi(theta);
    %theta = atan2(sin(theta), cos(theta));
end
