function R = eul2rotm(euler_angles)
    roll = euler_angles(1);
    pitch = euler_angles(2);
    yaw = euler_angles(3);

    Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];

    R = Rz * Ry * Rx;
end
