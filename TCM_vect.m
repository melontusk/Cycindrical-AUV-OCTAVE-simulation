function [Bout, Binv] = TCM_vect(CFG, R_r, L_r)
    Bout = [1, 0, 1, 0, 1, 0, 1, 0;
            0, -sin(CFG(1)), 0, -sin(CFG(1)), 0, sin(CFG(2)), 0, sin(CFG(2));
            0, cos(CFG(1)), 0, cos(CFG(1)), 0, -cos(CFG(2)), 0, -cos(CFG(2));
            0, R_r, 0, -R_r, 0, R_r, 0, -R_r;
            R_r*sin(CFG(1)), (L_r*cos(CFG(1)))/2, -R_r*sin(CFG(1)), (L_r*cos(CFG(1)))/2, -R_r*sin(CFG(2)), 0, R_r*sin(CFG(2)), 0;
            -R_r*cos(CFG(1)), (L_r*sin(CFG(1)))/2, R_r*cos(CFG(1)), (L_r*sin(CFG(1)))/2, R_r*cos(CFG(2)), 0, -R_r*cos(CFG(2)), 0];
    Binv = pinv(Bout);
end

function B = TCM_construct(CFG, R_r, L_r)
    BRa1 = [1, 0, 0; 0, cos(CFG(1)), -sin(CFG(1)); 0, sin(CFG(1)), cos(CFG(1))];
    BRa2 = [1, 0, 0; 0, cos(CFG(2)), -sin(CFG(2)); 0, sin(CFG(2)), cos(CFG(2))];
    a2RP = [0, 0, -1; 0, 1, 0; 1, 0, 0];
    PRb1 = PRb(CFG(3));
    PRb2 = PRb(CFG(4));
    PRb3 = PRb(CFG(5));
    PRb4 = PRb(CFG(6));
    B = zeros(6, 4);
    posP1 = BRa1*[-L_r/2; R_r; 0];
    posP2 = BRa1*[-L_r/2; -R_r; 0];
    posP3 = BRa2*[0; -R_r; 0];
    posP4 = BRa2*[0; R_r; 0];
    B(1:3, 1) = BRa1*PRb1*[1; 0; 0];
    B(1:3, 2) = BRa1*PRb2*[1; 0; 0];
    B(1:3, 3) = BRa2*a2RP*PRb3*[1; 0; 0];
    B(1:3, 4) = BRa2*a2RP*PRb4*[1; 0; 0];
    B(4:6, 1) = cross(posP1, BRa1*PRb1*[1; 0; 0]);
    B(4:6, 2) = cross(posP2, BRa1*PRb2*[1; 0; 0]);
    B(4:6, 3) = cross(posP3, BRa2*a2RP*PRb3*[1; 0; 0]);
    B(4:6, 4) = cross(posP4, BRa2*a2RP*PRb3*[1; 0; 0]);
end

function R = PRb(b)
    R = [cos(b), 0, sin(b); 0, 1, 0; -sin(b), 0, cos(b)];    
end