clear;
close all
clf

Ts = 0.00025;
sample_freq_hz = 1 / Ts;

PPR = 2048;
CPR = PPR * 4;
% Set up hypothetical velocity profile
max_speed_ms = 1.25;
diam_m = 0.25; 
max_speed_rs = 1.25 / pi / diam_m;

time = 0:Ts:5;
ss_min= round(1 * sample_freq_hz);
ss_max = round(4 * sample_freq_hz);

omega_hz = 1; % for velocity profile
omega_rs = omega_hz * 2 * pi;

pos_profile_r = 0.5 * max_speed_rs * time + ...
                0.5 * max_speed_rs / omega_rs * -1 * cos(omega_rs * time);
pos_profile_r = pos_profile_r - pos_profile_r(1);
counts = floor(pos_profile_r * CPR / 2 / pi);
%counts = tics(time, velocity_profile_rs, PPR);
velocity_profile_rs = max_speed_rs * (0.5 + 0.5 * sin(omega_rs * time));
acceleration_profile_rss = omega_rs * 0.5 * max_speed_rs * cos(omega_rs * time);
theta_obs_rad = 2 * pi / CPR * counts;


%% Search for an optimal smoothness parameter
smoothness_range = 0.98:0.000455:0.99999;
sse_vel = zeros(length(smoothness_range), 1);
sse_accel = zeros(length(smoothness_range), 1);

for i = 1:length(smoothness_range)
    smooth_param = smoothness_range(i);
    
    [pos_fit, goodness, out] = fit(time', theta_obs_rad','smoothingspline', 'SmoothingParam', smooth_param);
    [breaks,coefs,l,k,d] = unmkpp(pos_fit.p);
    dpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
    vel_spline_est = ppval(dpp, time);
    vel_error = vel_spline_est - velocity_profile_rs;
    sse_vel(i) = vel_error(ss_min:ss_max) * vel_error(ss_min:ss_max)';
    
    [breaks,coefs,l,k,d] = unmkpp(dpp);
    ddpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
    accel_spline_est = ppval(ddpp, time);
    accel_error = accel_spline_est - acceleration_profile_rss;
    sse_accel(i) = accel_error(ss_min:ss_max) * accel_error(ss_min:ss_max)';
    % asses errors in both estimates; 
end

yyaxis left
plot(smoothness_range, sse_vel);
ylim([0, inf]);
[min_sse_vel, best_param_vel] = min(sse_vel);
hold on;
plot(smoothness_range(best_param_vel), min_sse_vel, 'x');

xlabel('Smoothness parameter');
ylabel('SSE of Velocity Estimate');
yyaxis right
plot(smoothness_range, sse_accel);
ylim([0, inf]);
[min_sse_accel, best_param_accel] = min(sse_accel);
hold on;
plot(smoothness_range(best_param_accel), min_sse_accel, 'x');
ylabel('SSE of Acceleration Estimate');

figure;
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
plot(time, acceleration_profile_rss);
ylim([-10, 10]);
xlabel('Time [s]');
ylabel('Rotational Acceleration [rad/s^2]');
legend('Filtered Acceleration Estimate',...
        'Ground Truth Acceleration');