function R = PRb(b)
    % Rotation around y axes
    R = [cos(b), 0, sin(b); 0, 1, 0; -sin(b), 0, cos(b)];
end
