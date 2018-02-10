clear;
close all
clf

Ts = 0.001;
sample_freq_hz = 1 / Ts;

PPR = 2048;
% Set up hypothetical velocity profile
max_speed_ms = 1.25;
diam_m = 0.25; 
max_speed_rs = 1.25 / pi / diam_m;

time = 0:Ts:1.0;
velocity_profile_rs = 2 * max_speed_rs * [0:Ts:0.5-Ts, 0.5:-1 * Ts:0.0];

% encoder counts seen at each time step
counts = tics(time, velocity_profile_rs, PPR);
% corresponding radial position observations from counts
theta_obs_rad = 2 * pi / (4 * PPR) * counts;

[pos_fit, goodness, out] = fit(time', theta_obs_rad,'smoothingspline', 'SmoothingParam', 0.99995);
[breaks,coefs,l,k,d] = unmkpp(pos_fit.p);
dpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
vel_spline_est = ppval(dpp, time);

[breaks,coefs,l,k,d] = unmkpp(dpp);
ddpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
accel_spline_est = ppval(ddpp, time);



subplot(2, 1, 1);
plot(time, vel_spline_est);
hold on;
plot(time, velocity_profile_rs);
xlabel('Time [s]');
ylabel('Rotational Speed [rad/s]');
legend('Filtered Velocity Estimate',...
        'Ground Truth Rotational Speed');

    
subplot(2, 1, 2);
plot(time, accel_spline_est);
hold on;
plot([0, 0.4999, 0.50001, 1], 2 * max_speed_rs * [1, 1, -1, -1]);
ylim([-10, 10]);
xlabel('Time [s]');
ylabel('Rotational Acceleration [rad/s^2]');
legend('Filtered Acceleration Estimate',...
        'Ground Truth Acceleration');