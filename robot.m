function nudot = robot(tau, M, C, D, G, nu)
    nudot = M\(-(C + D)*nu - G + tau);
end