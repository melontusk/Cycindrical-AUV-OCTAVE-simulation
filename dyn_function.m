function [M_b,C_b,D,G] = dyn_function (R_r, L_r, m_r, rg, nu, nudot, eta)
    %% TODO
    %Calculer les coeff de D avec Cx et Sx
    %Pour les rotations: Kpp = 0 
    %% Physical constants
    
    rho = 1026; % Density of water
    g = 9.81; % Gravitationnal acceleration
    
    %% Robot parameters:
    %% Robot parameters:
    % R_r = 0.1; % Radius 
    % L_r = 0.4; % Length
    % m_r = 14; % Mass
    V_r = pi*R_r^2*L_r; % Volume, currently computes the cylinder's volume
    
    %rg = [0 0 0.5]; % Coordinates of G in Rb
    xg = rg(1);
    yg = rg(2);
    zg = rg(3);
    
    rb = [0 0 0]; % Coordinates of B in Rb
    xb = rb(1);
    yb = rb(2);
    zb = rb(3);

    
    angles = eta(4:6);
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);

    nu_1 = nu(1:3);
    nu_2 = nu(4:6);

    nudot_1 = nudot(1:3);
    nudot_2 = nudot(4:6);

    % Drag coefficients
    viscous = -[0.1 0.1 0.1 0.1 0.1 0.1];
    Xu = viscous(1);
    Yv = viscous(2);
    Zw = viscous(3);
    Kp = viscous(4);
    Mq = viscous(5);
    Nr = viscous(6);
    
    quadratic = -[0.1 0.1 0.1 0.1 0.1 0.1];
    Xuu = quadratic(1);
    Yvv = quadratic(2);
    Zww = quadratic(3);
    Kpp = quadratic(4);
    Mqq = quadratic(5);
    Nrr = quadratic(6);
    
    %% Matrices
    % Inertia
    % Inertia at center of mass
    I_cm = diag([1/2*m_r*R_r^2 1/12*m_r*L_r^2+1/4*m_r*R_r^2 1/12*m_r*L_r^2+1/4*m_r*R_r^2]);
    
    % Parallel axis theorem
    I_parr = [yg^2+zg^2 -xg*yg -xg*zg;
              -xg*yg xg^2+zg^2 -yg*zg;
              -xg*zg -yg*zg xg^2+yg^2];
    
    % Final inertia matrix
    I_b = I_cm + m_r*I_parr;
    
    % M_b = zeros(6);
    % M Matrix
    M_b = [m_r*eye(3) -m_r*S(rg);
            m_r*S(rg) I_b];
    
    % C Matrix
    C_b = [m_r*S(nu_2) -m_r*S(nu_2)*S(rg);
            m_r*S(rg)*S(nu_2) S(nu_2)*I_b];
    
    % G matrix, currently directly expressed as vector
    W = m_r*g; % mg
    % W = 0;
    B = rho*g*V_r; % ρg∇
    % B = 0;

    G = [(W-B)*sin(theta)
            -(W-B)*cos(theta)*sin(phi)
            -(W-B)*cos(theta)*cos(phi)
            -(yg*W-zb*B)*cos(theta)*cos(phi)+(zg*W-zb*B)*cos(theta)*sin(phi)
            (zg*W-zb*B)*sin(theta)+(xg*W-xb*B)*cos(theta)*cos(phi)
            -(xg*W-xb*B)*cos(theta)*sin(phi)-(yg*W-yb*B)*sin(theta)];
    
    % D matrix
    D = -diag([Xu Yv Zw Kp Mq Nr])-diag([Xuu*abs(nu(1)) Yvv*abs(nu(2)) Zww*abs(nu(3)) Kpp*abs(nu(4)) Mqq*(abs(nu(5))) Nrr*abs(nu(6))]);
    
    
    %% Masses ajoutées
    % Matrice M
    Xudot = viscous(1);
    Yv = viscous(2);
    Zw = viscous(3);
    Kp = viscous(4);
    Mq = viscous(5);
    Nr = viscous(6);
    
    %M_a = -[Xudot 0 0;
    %          0 Yvdot Yrdot;
      %        -xg*zg -yg*zg xg^2+yg^2];
    
    %% Compute complete model
    tau = M_b*nudot + C_b*nu + G + D*nu;
end

function s_out = S(inA)
    s_out = [0 -inA(3) inA(2);
            inA(3) 0 -inA(1);
            -inA(2) inA(1) 0];
end