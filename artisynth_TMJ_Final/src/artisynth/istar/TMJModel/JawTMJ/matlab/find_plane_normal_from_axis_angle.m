function plane_normal = find_plane_normal_from_axis_angle(axis, angle_deg)
    % Normalize the axis vector
    axis = axis / norm(axis);

    % Convert the angle to radians
    theta = deg2rad(angle_deg);

    % Create the axis-angle vector
    axang = [axis, theta];

    % Convert the axis-angle representation to a rotation matrix
    R = axang2rotm(axang);

    % Initial normal vector (assuming the plane starts as the xy-plane)
    initial_normal = [0; 0; 1];

    % Apply the rotation to the initial normal vector
    rotated_normal = R * initial_normal;

    % Output the resulting plane normal
    plane_normal = rotated_normal';
end


