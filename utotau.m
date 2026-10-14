function [tau, beta] = utotau(B, u, vect)
    % ctau = [tauc(1); tauc(2); tauc(4); tauc(6)];
    
    
    tau = B*u;
    if vect
        b1 = atan2(-u(2),u(1));
        b2 = atan2(-u(4),u(3));
        b3 = atan2(u(5),-u(6));
        b4 = atan2(u(7),-u(8));
    
        beta = [b1; b2; b3; b4];
    else
        beta = [0;0;0;0];
    end
    % tau = B*(Br\ctau);
    % tau = tauc;
end