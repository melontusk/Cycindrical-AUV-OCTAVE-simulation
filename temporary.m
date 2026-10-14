clear all;
close all;

[eta, eta_etoil, vect, beta, t, CFG] = sim_code(dt = 1e-1, eta_initial = [0; 0; 0; 0; 0; 0], nu_initial = [0; 0; 0; 0; 0; 0], vect = true, h_o = [1; 1; 1; 0; 0; 1], h_e = [1; 1; 1; 0; 0; 1]);

plotnewsina(eta, eta_etoil, vect, beta, t, CFG);


