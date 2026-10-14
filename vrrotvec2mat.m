function R = vrrotvec2mat(rotvec)
    axis = rotvec(1:3);
    theta = rotvec(4);

    axis = axis / norm(axis);

    x = axis(1);
    y = axis(2);
    z = axis(3);

    K = [0, -z, y; z, 0, -x; -y, x, 0];

    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end
