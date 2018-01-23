%clear;
close all
clf

Ts = 0.001;
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

% Estimator constants
Ts = 0.001; % 1 ms
sample_freq_hz = 1 / Ts;

b_filt_i = [1, 0];
a_filt_i = 1/Ts *[1, -1];

%[v_diff_filt_rs, Zf] = filter(b_filt_i{1}, a_filt_i{1}, theta_obs_rad, theta_obs_rad(1) * ones(length(a_filt_i{1}) - 1, 1));

% first diference:
v_first_diff = 1 / Ts * (theta_obs_rad(2:end) - theta_obs_rad(1:end - 1));
a_second_diff = 1 / Ts * (v_first_diff(2:end) - v_first_diff(1:end - 1));


%plot(time, v_diff_filt_rs);
%hold on;
subplot(2, 1, 1);
plot(time(2:end), v_first_diff);
hold on;
plot(time, velocity_profile_rs);
xlabel('Time [s]');
ylabel('Rotational Speed [rad/s]');
legend('Unfiltered Rotational Velocity Estimate',...
        'Ground Truth Rotational Speed');

subplot(2, 1, 2);
plot(time(3:end), a_second_diff);
hold on;
plot([0, 0.4999, 0.50001, 1], 2 * max_speed_rs * [1, 1, -1, -1]);
ylim([-10, 10]);
xlabel('Time [s]');
ylabel('Rotational Acceleration [rad/s^2]');
legend('Unfiltered Rotational Acceleration Estimate',...
        'Ground Truth Rotational Acceleration');


%legend('Filtered velocity', 'Unfiltered velocity', 'velocity profile');



