function [eta, eta_etoil, vect, beta, t, CFG] = sim_code(dt = 1e-1, eta_initial = [0; 0; 0; 0; 0; 0], nu_initial = [0; 0; 0; 0; 0; 0], vect = true, h_o = [1; 1; 1; 1; 1; 1], h_e = [1; 1; 1; 1; 1; 1], tracking_point = [0; 0; 0], CFG = [0, pi/2, 0, pi/2, pi/2, 0], m_r = 14, rg = [0 0 0.1], nuKp = 1e1, nuKi = 1e0, nuKd = 0, tauKp = 1e-2, tauKi = 0, R_r = 0.1, L_r = 0.4)

    meanerreta = zeros(6, 1);
    meanerrnu = zeros(6, 1);
    meanerrnui = zeros(6, 1);
    meanerretai = zeros(6, 1);


    t = 0:dt:100;
    eta_etoil = [0.; 0.; 0.; 0; 0.; 0.];
    etadot_etoil = [0.; 0.; 0.; 0.; 0.; 0.];
    eta = eta_initial;
    etadot = [0.; 0.; 0.; 0.; 0.; 0.];
    erreta = [0.; 0.; 0.; 0.; 0.; 0.];
    erreta_robot = [0.; 0.; 0.; 0.; 0.; 0.];
    erretai = [0.; 0.; 0.; 0.; 0.; 0.];
    nuc = [0.; 0.; 0.; 0.; 0.; 0.];
    nucdot = [0.; 0.; 0.; 0.; 0.; 0.];
    nu = nu_initial;
    nudot = [0.; 0.; 0.; 0.; 0.; 0.];
    errnui = [0.; 0.; 0.; 0.; 0.; 0.];
    errnu = [0.; 0.; 0.; 0.; 0.; 0.];
    errnu_robot = [0.; 0.; 0.; 0.; 0.; 0.];
    tauc = [0.; 0.; 0.; 0.; 0.; 0.];
    tau = [0.; 0.; 0.; 0.; 0.; 0.];
    beta = [0.; 0.; 0.; 0.];
    if vect
        u = [0.; 0.; 0.; 0.; 0.; 0.; 0.; 0.];
    else
        u = [0.;0.;0.;0.];
    end
    [M,C,D,G] = dyn_function (R_r, L_r, m_r, rg, nu, nudot, eta);

    temp = t;
    index = 0;

    for ti = t
        index  = index + 1;
        %% Desired Position
        x = ti/10;
        y = sin(x);
        z = 0;

        vx = (x - eta_etoil(1, end))/dt;
        vy = (y - eta_etoil(2, end))/dt;
        vz = (z - eta_etoil(3, end))/dt;

        [neta2_etoil, neta1_etoil] = generateTrajectory(x, y, z, vx, vy, vz);
        eta_etoil = [eta_etoil, [neta1_etoil;neta2_etoil]];

        etadotetoil = (eta_etoil(:, end) - eta_etoil(:, end-1))/dt;
        etadot_etoil = [etadot_etoil, etadotetoil];

        nerretadot = (etadot_etoil(:, end) - etadot(:, end));

        nerreta = (eta_etoil(:, end) - eta(:, end));
        erreta = [erreta, nerreta];

        nerretai = erretai(:, end) + nerreta*dt;
        erretai = [erretai, nerretai];

        %% Command

        [nnuc, erretarobot] = command(tracking_point, etadotetoil, nerreta, nerretai, eta(:, end), nuKp, nuKi, nuKd, h_o, h_e, nerretadot);
        erreta_robot = [erreta_robot, erretarobot];
        nuc = [nuc, nnuc];

        nnucdot = (nuc(:, end) - nuc(:, end-1))/dt;
        nucdot = [nucdot, nnucdot];

        nerrnu = (nnuc - nu(:, end))/dt;
        errnu = [errnu, nerrnu];

        nerrnui = errnui(:, end) + nerrnu*dt;
        errnui = [errnui, nerrnui];

        [ntauc, errnurobot] = tau_command(nnucdot,nerrnui, nerrnu, nnuc, tauKp, tauKi, M, C, D, G);
        errnu_robot = [errnu_robot, errnurobot];
        tauc = [tauc, ntauc];

        if vect
            [B, Binv] = TCM_vect(CFG, R_r, L_r);

            newu = thrust(Binv, ntauc);
            u = [u, newu];

            [ntau, nbeta] = utotau(B, newu, vect);
            tau = [tau, ntau];
            beta = [beta, nbeta];
        else
            [B, Binv] = TCM(h_e, CFG, R_r, L_r);
            newu = thrust(Binv, ntauc);
            u = [u, newu];

            [ntau, nbeta] = utotau(B, newu, vect);
            tau = [tau, ntau];
            beta = [beta, nbeta];
        end

        %% Robot

        nnudot = robot(ntau, M, C, D, G, nu(:, end));
        nudot = [nudot, nnudot];


        nnu = nu(:, end) + nnudot*dt;
        nu = [nu, nnu];

        netadot = Transform(nnu, eta(:, end));
        etadot = [etadot, netadot];

        neta = eta(:, end) + netadot*dt;
        eta = [eta, neta];

        [M,C,D,G] = dyn_function (R_r, L_r, m_r, rg, nnu, nnudot, neta);



    end

    meanerreta = [meanerreta, mean(abs(erreta'))'];
    meanerrnu = [meanerrnu, mean(abs(errnu'))'];
    meanerrnui = [meanerrnui, mean(abs(errnui'))'];
    meanerretai = [meanerretai, mean(abs(erretai'))'];

    figure
    eta1 = eta(1, :);
    eta2 = eta(2, :);
    eta3 = eta(3, :);

    plot3(eta1, eta2, eta3);

    etae1 = eta_etoil(1, :);
    etae2 = eta_etoil(2, :);
    etae3 = eta_etoil(3, :);
    hold on

    plot3(etae1, etae2, etae3, '--r');
    title("Trajectory")
    legend({"Simulated Path", "Desired Path"})

    figure
    plot(t, erreta(:, 2:end))
    title('error \eta')
    xlabel('Time')
    ylabel('error \eta (m)')
    legend({'\epsilon_{\eta_1}', '\epsilon_{\eta_2}', '\epsilon_{\eta_3}', '\epsilon_{\eta_4}', '\epsilon_{\eta_5}', '\epsilon_{\eta_6}'})
    legend

    figure
    plot(t, erretai(:, 2:end))
    title('error \eta integral')
    xlabel('Time')
    ylabel('error \eta (m)')
    legend({'\epsilon_{\eta_{i1}}', '\epsilon_{\eta_{i2}}', '\epsilon_{\eta_{i3}}', '\epsilon_{\eta_{i4}}', '\epsilon_{\eta_{i5}}', '\epsilon_{\eta_{i6}}', })
    legend

    figure
    plot(t, errnu(:, 2:end))
    title('error \nu')
    xlabel('Time')
    ylabel('error \nu (m/s)')
    legend({'\epsilon_{\nu_1}', '\epsilon_{\nu_2}', '\epsilon_{\nu_3}', '\epsilon_{\nu_4}', '\epsilon_{\nu_5}', '\epsilon_{\nu_6}'})
    legend

    if vect
        figure
        plot(t, u(:, 2:end))
        title("Virtual thrusters output")
        xlabel('Time')
        ylabel('Input(N)')
        legend({'u_{1x}','u_{1z}', 'u_{2x}', 'u_{2z}', 'u_{3x}', 'u_{3y}', 'u_{4x}', 'u_{4y}'})
        grid on
    else
        figure
        plot(t, u(:, 2:end))
        title("Thrusters output")
        xlabel('Time')
        ylabel('Input(N)')
        legend({'u_{1}','u_{2}', 'u_{3}', 'u_{4}'})
        grid on

    figure
    plot(t, nuc(:, 2:end))
    title('Desired \nu')
    xlabel('Time')
    ylabel('Speed(m/s)')
    legend({'\nu_r1', '\nu_r2', '\nu_r3', '\nu_r4', '\nu_r5', '\nu_r6'})
    grid on
end
