function [tauc, errnu] = tau_command(nucdot, errnui, errnu, nu, Kp, Ki, M, C, D, G)
    % errnu = zeros(6, 1);
    tauc = M*(nucdot + Kp*errnu + Ki*errnui) + C*nu + G + D*nu;
    % taumax = 40;
    % tauc(3) = 0;
    % tauc(5) = 0;
    % for i = 1:6
    %     if tauc(i) > taumax
    %         tauc(i) = taumax;
    %     end
    %     if tauc(i) < -taumax
    %         tauc(i) = -taumax;
    %     end
    % end
    
end