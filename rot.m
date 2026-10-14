function [x_rotated, y_rotated, z_rotated, rotationMatrix] = rot(roll, pitch, yaw)
    eulerAngles = [roll pitch yaw];
    rotationMatrix = eul2rotm(eulerAngles);
    x_axis = [1; 0; 0];
    y_axis = [0; 1; 0];
    z_axis = [0; 0; 1];
    x_rotated = rotationMatrix * x_axis;
    y_rotated = rotationMatrix * y_axis;
    z_rotated = rotationMatrix * z_axis;
end
