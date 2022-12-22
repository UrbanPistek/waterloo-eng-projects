% plot ct

figure(1);
plot(t_sim, r_sim);
hold on 
plot(t_sim, y_sim);
hold on 
plot(t_sim, u_sim);
legend("r(t)", "y(t)", "u(t)");
