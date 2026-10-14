function [x_cyl_final, y_cyl_final, z_cyl_final] = rotatecyl(target_axis, x_cyl, y_cyl, z_cyl, x_center, y_center, z_center)
    cyl_coords = [x_cyl(:)'; y_cyl(:)'; z_cyl(:)'];
    default_axis = [0, 0, 1];
    rot_cyl = vrrotvec2mat(vrrotvec(default_axis, target_axis));
    rotated_coords = rot_cyl * cyl_coords;
    x_cyl_rotated = reshape(rotated_coords(1,:), size(x_cyl));
    y_cyl_rotated = reshape(rotated_coords(2,:), size(y_cyl));
    z_cyl_rotated = reshape(rotated_coords(3,:), size(z_cyl));
    x_cyl_final = x_cyl_rotated + x_center;
    y_cyl_final = y_cyl_rotated + y_center;
    z_cyl_final = z_cyl_rotated + z_center;
end
