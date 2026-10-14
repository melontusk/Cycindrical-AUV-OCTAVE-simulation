function J = build_J(eta)
    phi = eta(4);
    theta = eta(5);
    psi = eta(6);
    Rz = [cos(psi), sin(psi), 0;
          -sin(psi), cos(psi), 0;
          0, 0, 1];
    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
    Rx = [1, 0, 0;
          0, cos(phi), sin(phi);
          0, -sin(phi), cos(phi)];
    J1 = Rz*Ry*Rx;
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi), -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    J = [J1, zeros(3); zeros(3), J2];
end