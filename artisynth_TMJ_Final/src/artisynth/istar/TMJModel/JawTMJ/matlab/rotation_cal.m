axis= [0.95058, 0.31047, 0];
angle = 94.205 ;
alpha = 90 ;
[new_axis, new_angle] = rotate_axis_angle_around_local_y(axis,angle,alpha);
plane_normal = find_plane_normal_from_axis_angle(new_axis, new_angle)
