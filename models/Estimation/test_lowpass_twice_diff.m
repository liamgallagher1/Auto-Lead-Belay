%clear;
%close all
%clf

Ts = 0.001;
sample_freq_hz = 1 / Ts;

PPR = 2048;
% Set up hypothetical velocity profile
max_speed_ms = 1.25;
diam_m = 0.25; 
max_speed_rs = 1.25 / pi / diam_m;

time = 0:Ts:1.0;
velocity_profile_rs = 2 * max_speed_rs * [0:0.001:0.499, 0.5:-0.001:0.0];

% encoder counts seen at each time step
counts = tics(time, velocity_profile_rs, PPR);
% corresponding radial position observations from counts
theta_obs_rad = 2 * pi / (4 * PPR) * counts;

% Get velocity estimate from position signal
[vel_filt_rs, Zf] = filter(b_filt_vel_i{1}, a_filt_vel_i{1}, theta_obs_rad);
b_filt_vel_i{1}
a_filt_vel_i{1}
% Get acceleration from filtered velocity
[a_filt_rss, Zff] = filter(b_filt_ii_vel{1}, a_filt_ii_vel{1}, vel_filt_rs);

subplot(2, 1, 1);
plot(time, vel_filt_rs);
hold on;
plot(time, velocity_profile_rs);
xlabel('Time [s]');
ylabel('Rotational Speed [rad/s]');
legend('Filtered Velocity Estimate',...
        'Ground Truth Rotational Speed');

    
subplot(2, 1, 2);
plot(time, a_filt_rss);
hold on;
plot([0, 0.4999, 0.50001, 1], 2 * max_speed_rs * [1, 1, -1, -1]);
ylim([-10, 10]);
xlabel('Time [s]');
ylabel('Rotational Acceleration [rad/s^2]');
legend('Filtered Acceleration Estimate',...
        'Ground Truth Acceleration');