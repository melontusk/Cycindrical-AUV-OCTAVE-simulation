clear all;
close all;

[eta, eta_etoil, vect, beta, t, CFG] = sim_code();

plotnewsina(eta, eta_etoil, vect, beta, t, CFG);


