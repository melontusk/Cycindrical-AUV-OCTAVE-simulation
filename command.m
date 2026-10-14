function [nuc, erreta] = command(Tracking_point, etadotetoilin, erretain, erretaiin, eta, Kp, Ki, Kd, h_o, h_e, erretadot)
    
    etadotetoil = etadotetoilin;
    erreta = erretain;
    erretai = erretaiin;
    
    % index = 0;
    % for i = h_e'
    %     index = index + 1;
    %     if i == 0
    %         erreta(index) = 0;
    %         erretai(index) = 0;
    %         etadotetoil(index) = 0; 
    %     end
    % end

    erreta = erreta.*h_e;
    erretai = erretai.*h_e;
    erretadot = erretadot.*h_e;
    % etadotetoil = erretadoetoil.*h_e;

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
    
    ex = Tracking_point(1);
    ey = Tracking_point(2);
    ez = Tracking_point(3); 
    T = [1, 0, 0, 0, ez, -ey; 0, 1, 0, -ez, 0, ex; 0, 0, 1, ey, -ex, 0;
         0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 1, 0; 0, 0, 0, 0, 0, 1];
    H = handy(Tracking_point, h_o, h_e);

    nuc = H*inv(T)*inv(J)*(etadotetoil + Kp*erreta + Ki*erretai + Kd*erretadot);
    
    % numax = 1;
    % for i = 1:length(nuc)
    %     if nuc(i) > numax
    %         nuc(i) = numax;
    %     end
    %     if nuc(i) < -numax
    %         nuc(i) = -numax;
    %     end
    % end
    % index = 0;
    % for i = h_e'
    %     index = index + 1;
    %     if i == 0
    %         nuc(index) = 0; 
    %     end
    % end
end

function H = handy(Tracking_point, h_o, h_e)
    H = eye(6);
    eps = ones(3, 1);
    for k = 1:3
        if Tracking_point(k) ~= 0
            eps(k) = 1/Tracking_point(k);
        end
    end
    somme = S(eps);
    for i = 4:6
        if h_o(i) == 1 && h_e(i) == 0
            for j = [1:3]
                if h_e(j) == 1 && h_o(j) == 0 && j ~= i-3
                    H(i, j) = somme(i-3, j);
                    H(j, :) = zeros(1, 6);
                end
            end
        end
    end
end

function s_out = S(inA)
    s_out = [0 -inA(3) inA(2);
            inA(3) 0 -inA(1);
            -inA(2) inA(1) 0];
end