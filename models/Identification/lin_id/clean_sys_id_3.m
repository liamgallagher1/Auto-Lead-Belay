clear;

%M = csvread('logs/early_morning_long_sys_id96_5_45.csv', 1, 0);
M = csvread('logs/harder_but_slowest_96_6_18.csv', 1, 0);


CPR = 8192;
v_max = 36;

Ts = 0.0005;
time = M(:, 1) + 10^-9 * M(:, 2);
time = time - time(1);
indx = time > 0; %29.5;

time = time(indx);
M = M(indx, :);

lm_raw_amp = M(:, 8:9);
%lm_current = M(:, 12); % Noisy estimate of the current
indx = (time > 11 & time < 15) | (time > 26 & time < 30);
bias = mean(lm_raw_amp(indx, 1));
lm_current = (lm_raw_amp(:, 1) - bias) * 3.3 / 4096 / 0.0185;

lm_count   = M(:, 14); % Encoder counts
lm_v = v_max * M(:, 19); % signed, in volts
lm_theta = lm_count * 2 * pi / CPR;
load('filter_3.mat');

lm_omega = filtfilt(b_filt_vel_i{1},a_filt_vel_i{1}, lm_theta);


% best_smoothness = 0.9995;
% num_segments = 50;
% [coefs, start_indx, end_indx, breaks] = ...
%      fast_smooth(time, lm_theta, best_smoothness, num_segments);
% first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
% first_derv_coefs = coefs * first_derv_mat;
% first_derv_fit = mkpp(breaks, first_derv_coefs);
% subtime = breaks(1):Ts:breaks(end);
% omega = ppval(breaks(1):Ts:breaks(end), first_derv_fit);
 
subplot(3, 1, 1);
plot(time, lm_theta);
subplot(3, 1, 2);
plot(time, lm_current);
subplot(3, 1, 3);
plot(time, lm_v);

% input = lm_v;
% output = [lm_theta, lm_current];

figure;
width = 2.5; % rad/s
smooth_sign = tanh(lm_omega / width);

subplot(3, 1, 1);
plot(time, lm_omega);
subplot(3, 1, 2);
plot(time, tanh(lm_omega / width));
subplot(3, 1, 3);
range_max = 100;
vel_range = -1 * range_max:range_max / 1000:range_max;
plot(vel_range, tanh(vel_range / width));

input = [lm_v, tanh(lm_omega / width)];
output = [lm_theta, lm_current];
data = iddata(output, input, Ts);

data.TimeUnit = 'seconds';
% Set names of input channels
ze.InputName = {'Voltage','Sign of velocity (sigmoid)'};
% Set units for input variables
ze.InputUnit = {'V','[-1, 1]'};
% Set name of output channel
ze.OutputName = {'Theta', 'Current'};
% Set unit of output channel
ze.OutputUnit = {'Radians', 'Ampere'};

save('logs/more_filtered_sigmoid.mat', 'data');





