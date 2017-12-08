clear;
close all

theta_init = 0;
theta_dot_init = 0;
r_s_init = 0 * 0.0254; % 0 in, [m]
r_dot_init = 0;

Y_init = [theta_init; theta_dot_init; r_s_init; r_dot_init];

[t, Y] = ode45(@ode_func_no_coriolis, 0:0.001:1, Y_init);
theta_t = Y(:, 1);
omega_t = Y(:, 2);
r_t = Y(:, 3);
r_dot_t = Y(:, 4);

subplot(2, 1, 1);
plot(t, omega_t);
ylabel('omega [rad/s]');
xlabel('time [s]');

subplot(2, 1, 2);
plot(t, r_t);
ylabel('spring length [m]');
xlabel('time [s]');