function u = thrust(Binv, tauc)
    
    u = Binv*tauc;
    
    umax = 40;
    for i = 1:length(u)
        if u(i) > umax
            u(i) = umax;
        end
        if u(i) < -umax
            u(i) = -umax;
        end
    end


    % tau = B*u;
    
    % tau = B*(Br\ctau);
    % tau = tauc;
end
