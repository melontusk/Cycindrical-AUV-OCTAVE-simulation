function [orientation, position_NED]= generateTrajectory(x, y, z, vx, vy, vz)
    yaw = atan2(vy, vx);            
    pitch = atan2(-vz, sqrt(norm(vx) + norm(vy))); 
    roll = 0.0;

    position_NED = [x;y;z];

    orientation = [roll;pitch;yaw];
end