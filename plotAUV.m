function plotAUV(eta, eta_etoil, vect, beta, t, CFG)
    n = 2e1;            % speed
    temp = 5;           % camera position offset
    zoom = 5;           % camera zoom
    distx = 1000;       % x-axis length
    disty = 1000;       % y-axis length
    distz = 1000;       % z-axis length
    scale = 0.15;          % robot_axis scale
    scale_thrust = 0.15; % thrust axis scale
    scale_robot = 0;    % robot_size
    resolution = 20;    % 3D robot resolution

    % Load logged data
    eta1 = eta(1, :);
    eta2 = eta(2, :);
    eta3 = eta(3, :);
    eta4 = eta(4, :);
    eta5 = eta(5, :);
    eta6 = eta(6, :);
    etae1 = eta_etoil(1, :);
    etae2 = eta_etoil(2, :);
    etae3 = eta_etoil(3, :);

    run_time = 20;
    time_step = 1e-3;
    nuKi = 1e-2;
    nuKp = 1e0;
    tauKp = 1e2;
    R_r = 0.1;
    L_r = 0.4;
    l_r = 0.4;
    r_r = 0.1;
    rg = [0 0 0.1];
    Track_point = [0., 0., 0.];

    if vect
        b1 = beta(1, :);
        b2 = beta(2, :);
        b3 = beta(3, :);
        b4 = beta(4, :);
    end

    % Initialize 3D plot
    figure;
    plot3(etae1, etae2, etae3, "--");
    hold on
    plot3(eta1, eta2, eta3, "--r");
    h = plot3(eta1(1), eta2(1), eta3(1), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'cyan');

    [x_rotated, y_rotated, z_rotated, rotationMatrix] = rot(eta4(1), eta5(1), eta6(1));
    h_x = quiver3(eta1(1), eta2(1), eta3(1), scale*x_rotated(1), scale*x_rotated(2), scale*x_rotated(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.2);
    h_y = quiver3(eta1(1), eta2(1), eta3(1), scale*y_rotated(1), scale*y_rotated(2), scale*y_rotated(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.2);
    h_z = quiver3(eta1(1), eta2(1), eta3(1), scale*z_rotated(1), scale*z_rotated(2), scale*z_rotated(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.2);

    a2RP = [0, 0, -1; 0, 1, 0; 1, 0, 0];
    if vect
        PRb1 = PRb(b1(1));
        PRb2 = PRb(b2(1));
        PRb3 = PRb(b3(1));
        PRb4 = PRb(b4(1));
    else
        PRb1 = PRb(CFG(3));
        PRb2 = PRb(CFG(4));
        PRb3 = PRb(CFG(5));
        PRb4 = PRb(CFG(6));
    end
    BRa1 = [1, 0, 0; 0, cos(CFG(1)), -sin(CFG(1)); 0, sin(CFG(1)), cos(CFG(1))];
    BRa2 = [1, 0, 0; 0, cos(CFG(2)), -sin(CFG(2)); 0, sin(CFG(2)), cos(CFG(2))];
    x_b1 = BRa1*PRb1*[1; 0; 0];
    x_b2 = BRa1*PRb2*[1; 0; 0];
    x_b3 = BRa2*a2RP*PRb3*[1; 0; 0];
    x_b4 = BRa2*a2RP*PRb4*[1; 0; 0];

    radius = R_r*10^scale_robot;
    height = L_r*10^scale_robot;
    radius_thrust = 0.05*10^scale_robot;
    height_thrust = 0.12*10^scale_robot;
    [x_cyl, y_cyl, z_cyl] = cylinder(radius, resolution);
    [x_cylth, y_cylth, z_cylth] = cylinder(radius_thrust, resolution);
    z_cyl = height * z_cyl - height/2;
    z_cylth = height_thrust*z_cylth - height_thrust/2;
    [x_cyl_final, y_cyl_final, z_cyl_final] = rotatecyl(x_rotated, x_cyl, y_cyl, z_cyl, eta1(1), eta2(1), eta3(1));
    pos1 = BRa1*[-height*l_r/L_r/2; -radius - radius_thrust; 0];
    pos2 = BRa1*[-height*l_r/L_r/2; radius + radius_thrust; 0];
    pos3 = BRa2*[0; radius + radius_thrust; 0];
    pos4 = BRa2*[0; -radius - radius_thrust; 0];

    h_b1 = quiver3(pos1(1), pos1(2), pos1(3), scale_thrust*x_b1(1), scale_thrust*x_b1(2), scale_thrust*x_b1(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.2);
    h_b2 = quiver3(pos2(1), pos2(2), pos2(3), scale_thrust*x_b2(1), scale_thrust*x_b2(2), scale_thrust*x_b2(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.2);
    h_b3 = quiver3(pos3(1), pos3(2), pos3(3), scale_thrust*x_b3(1), scale_thrust*x_b3(2), scale_thrust*x_b3(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.2);
    h_b4 = quiver3(pos4(1), pos4(2), pos4(3), scale_thrust*x_b4(1), scale_thrust*x_b4(2), scale_thrust*x_b4(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.2);

    [x_cylth1_final, y_cylth1_final, z_cylth1_final] = rotatecyl(x_b1, x_cylth, y_cylth, z_cylth, pos1(1), pos1(2), pos1(3));
    [x_cylth2_final, y_cylth2_final, z_cylth2_final] = rotatecyl(x_b2, x_cylth, y_cylth, z_cylth, pos2(1), pos2(2), pos2(3));
    [x_cylth3_final, y_cylth3_final, z_cylth3_final] = rotatecyl(x_b3, x_cylth, y_cylth, z_cylth, pos3(1), pos3(2), pos3(3));
    [x_cylth4_final, y_cylth4_final, z_cylth4_final] = rotatecyl(x_b4, x_cylth, y_cylth, z_cylth, pos4(1), pos4(2), pos4(3));

    h_cyl = surf(x_cyl_final, y_cyl_final, z_cyl_final, 'FaceColor', 'cyan', 'EdgeColor', 'none');
    h_cylth1 = surf(x_cylth1_final, y_cylth1_final, z_cylth1_final, 'FaceColor', 'red', 'EdgeColor', 'none');
    h_cylth2 = surf(x_cylth2_final, y_cylth2_final, z_cylth2_final, 'FaceColor', 'red', 'EdgeColor', 'none');
    h_cylth3 = surf(x_cylth3_final, y_cylth3_final, z_cylth3_final, 'FaceColor', 'red', 'EdgeColor', 'none');
    h_cylth4 = surf(x_cylth4_final, y_cylth4_final, z_cylth4_final, 'FaceColor', 'red', 'EdgeColor', 'none');

    axis ([-distx distx -disty disty -distz distz])
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('3D Animation results');

    % Animate
    for k = 1:length(t)/n
        % eta3(k*n) = 0;
        set(h, 'XData', eta1(k*n), 'YData', eta2(k*n), 'ZData', eta3(k*n));
        camtarget([eta1(k*n), eta2(k*n), eta3(k*n)]);
        campos([eta1(k*n)-2*temp, eta2(k*n)-2*temp, eta3(k*n)+temp]);
        camva(zoom)

        [x_rotated, y_rotated, z_rotated, rotationMatrix] = rot(eta4(k*n), eta5(k*n), eta6(k*n));
        set(h_x, 'XData', eta1(k*n), 'YData', eta2(k*n), 'ZData', eta3(k*n), 'UData', scale*x_rotated(1), 'VData', scale*x_rotated(2), 'WData', scale*x_rotated(3));
        set(h_y, 'XData', eta1(k*n), 'YData', eta2(k*n), 'ZData', eta3(k*n), 'UData', scale*y_rotated(1), 'VData', scale*y_rotated(2), 'WData', scale*y_rotated(3));
        set(h_z, 'XData', eta1(k*n), 'YData', eta2(k*n), 'ZData', eta3(k*n), 'UData', scale*z_rotated(1), 'VData', scale*z_rotated(2), 'WData', scale*z_rotated(3));

        [x_cyl_rotated, y_cyl_rotated, z_cyl_rotated] = rotatecyl(x_rotated, x_cyl, y_cyl, z_cyl, eta1(k*n), eta2(k*n), eta3(k*n));
        posr1 = rotationMatrix*pos1;
        posr2 = rotationMatrix*pos2;
        posr3 = rotationMatrix*pos3;
        posr4 = rotationMatrix*pos4;
        if vect
            PRb1 = PRb(b1(k*n));
            PRb2 = PRb(b2(k*n));
            PRb3 = PRb(b3(k*n));
            PRb4 = PRb(b4(k*n));
        else
            PRb1 = PRb(CFG(3));
            PRb2 = PRb(CFG(4));
            PRb3 = PRb(CFG(5));
            PRb4 = PRb(CFG(6));
        end
        x_b1 = BRa1*PRb1*[1; 0; 0];
        x_b2 = BRa1*PRb2*[1; 0; 0];
        x_b3 = BRa2*a2RP*PRb3*[1; 0; 0];
        x_b4 = BRa2*a2RP*PRb4*[1; 0; 0];

        rx_b1 = rotationMatrix*x_b1;
        rx_b2 = rotationMatrix*x_b2;
        rx_b3 = rotationMatrix*x_b3;
        rx_b4 = rotationMatrix*x_b4;

        set(h_b1, 'XData', eta1(k*n) + posr1(1), 'YData', eta2(k*n) + posr1(2), 'ZData', eta3(k*n) + posr1(3), 'UData', scale_thrust*rx_b1(1), 'VData', scale_thrust*rx_b1(2), 'WData', scale_thrust*rx_b1(3));
        set(h_b2, 'XData', eta1(k*n) + posr2(1), 'YData', eta2(k*n) + posr2(2), 'ZData', eta3(k*n) + posr2(3), 'UData', scale_thrust*rx_b2(1), 'VData', scale_thrust*rx_b2(2), 'WData', scale_thrust*rx_b2(3));
        set(h_b3, 'XData', eta1(k*n) + posr3(1), 'YData', eta2(k*n) + posr3(2), 'ZData', eta3(k*n) + posr3(3), 'UData', scale_thrust*rx_b3(1), 'VData', scale_thrust*rx_b3(2), 'WData', scale_thrust*rx_b3(3));
        set(h_b4, 'XData', eta1(k*n) + posr4(1), 'YData', eta2(k*n) + posr4(2), 'ZData', eta3(k*n) + posr4(3), 'UData', scale_thrust*rx_b4(1), 'VData', scale_thrust*rx_b4(2), 'WData', scale_thrust*rx_b4(3));

        [x_cylth1_final, y_cylth1_final, z_cylth1_final] = rotatecyl(rotationMatrix*x_b1, x_cylth, y_cylth, z_cylth, eta1(k*n)+posr1(1), eta2(k*n)+posr1(2), eta3(k*n)+posr1(3));
        [x_cylth2_final, y_cylth2_final, z_cylth2_final] = rotatecyl(rotationMatrix*x_b2, x_cylth, y_cylth, z_cylth, eta1(k*n)+posr2(1), eta2(k*n)+posr2(2), eta3(k*n)+posr2(3));
        [x_cylth3_final, y_cylth3_final, z_cylth3_final] = rotatecyl(rotationMatrix*x_b3, x_cylth, y_cylth, z_cylth, eta1(k*n)+posr3(1), eta2(k*n)+posr3(2), eta3(k*n)+posr3(3));
        [x_cylth4_final, y_cylth4_final, z_cylth4_final] = rotatecyl(rotationMatrix*x_b4, x_cylth, y_cylth, z_cylth, eta1(k*n)+posr4(1), eta2(k*n)+posr4(2), eta3(k*n)+posr4(3));

        set(h_cyl, 'XData', x_cyl_rotated, 'YData', y_cyl_rotated, 'ZData', z_cyl_rotated, 'FaceAlpha', 0.5);
        set(h_cylth1, 'XData', x_cylth1_final, 'YData', y_cylth1_final, 'ZData', z_cylth1_final, 'FaceAlpha', 0.5);
        set(h_cylth2, 'XData', x_cylth2_final, 'YData', y_cylth2_final, 'ZData', z_cylth2_final, 'FaceAlpha', 0.5);
        set(h_cylth3, 'XData', x_cylth3_final, 'YData', y_cylth3_final, 'ZData', z_cylth3_final, 'FaceAlpha', 0.5);
        set(h_cylth4, 'XData', x_cylth4_final, 'YData', y_cylth4_final, 'ZData', z_cylth4_final, 'FaceAlpha', 0.5);
        gif_filename = 'robot_animation.gif';
        drawnow;
    end
end
