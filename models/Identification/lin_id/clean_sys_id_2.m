clear;

M = csvread('logs/1_direction_other_way_96_2_15.csv', 1, 0);
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
indx = (time > 11 & time < 15) | (time < 26 & time < 30);
bias = mean(lm_raw_amp(indx, 1));
lm_current = (lm_raw_amp(:, 1) - bias) * 3.3 / 4096 / 0.0185;

lm_count   = M(:, 14); % Encoder counts
lm_v = v_max * M(:, 19); % signed, in volts
lm_theta = lm_count * 2 * pi / CPR;


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

subplot(2, 1, 1);
plot(time, lm_current);
subplot(2, 1, 2);
plot(time, lm_v);


input = [lm_v, ones(length(lm_v), 1)];
output = [lm_theta, lm_current];
data = iddata(output, input, Ts);

save('logs/one_dir_data.mat', 'data');





