function rotvec = vrrotvec(a, b)
    a = a(:) / norm(a);
    b = b(:) / norm(b);

    v = cross(a, b);
    s = norm(v);
    c = dot(a, b);

    if s == 0
        if c < 0
            if abs(a(1)) < 0.9
                perp = cross(a, [1; 0; 0]);
            else
                perp = cross(a, [0; 1; 0]);
            end
            perp = perp / norm(perp);
            rotvec = [perp(1); perp(2); perp(3); pi];
        else
            rotvec = [0; 0; 0; 0];
        end
    else
        axis = v / s;
        theta = atan2(s, c);
        rotvec = [axis(1); axis(2); axis(3); theta];
    end
end
