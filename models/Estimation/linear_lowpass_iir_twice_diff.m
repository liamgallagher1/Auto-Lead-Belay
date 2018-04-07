clear;
close all;

% http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4063529
Ts = 0.0005;
sample_freq_hz = 1 / Ts;

% Velocity filter params
% was 150
passband_freq_vel_hz = 100;
rel_cutoff_rad_vel = 2 * passband_freq_vel_hz / sample_freq_hz; 
vel_order = 10;
stopband_attenuation_vel_db = 100; 

% Acceleration filter params
passband_freq_accel_hz = 30;
rel_cutoff_rad_accel = 2 * passband_freq_accel_hz / sample_freq_hz; 
accel_order = 8;
stopband_attenuation_accel_db = 40; 

% First order wide band diff: Al-Alaoui operator
% appropriate version
diff_i = 8 / 7 / Ts * tf([1, -1], [1, 1/7], Ts);
% unitless version or something
diff_i_ul = 0.3638 * tf([ 1, -1], [1, 1 / 7], Ts);
% other worse version also presented in paper
% diff_ii_ul = 0.25586 * tf([1, 0, -1], [1, 0.5358, 0.0718], Ts);

%% Velocity Filter

% Velocity Filter
[b_lp_i, a_lp_i] = cheby2(vel_order, stopband_attenuation_vel_db, rel_cutoff_rad_vel);

% plot velocity filter
freqz(b_lp_i, a_lp_i);
title('Low-Pass Filter for Velocity');
figure;

% Filter on velocity
filt_lp_i = tf(b_lp_i, a_lp_i, Ts);

% differentiation and filtering for velocity
diff_filter_i = diff_i * filt_lp_i;
% Unitless version for plotting
diff_filter_i_ul = diff_i_ul * filt_lp_i;
%diff_filter_ii_ul = diff_ii_ul * filt_lp_i * diff_i_ul;

% Differentiator for comparison
diff_c = tf([1, 0], 1);
diff_d = c2d(diff_c, Ts, 'tustin');
[b_diff, a_diff] = tfdata(diff_d);

[b_filt_vel_i, a_filt_vel_i] = tfdata(diff_filter_i);
[b_filt_i_ul, a_filt_i_ul] = tfdata(diff_filter_i_ul);


%% Plot Velocity Filter
subplot(3, 1, 1);
[h_i, w_i] = freqz(b_filt_i_ul{1}, a_filt_i_ul{1},'whole',2001);

plot(w_i/pi, abs(h_i));
hold on;
plot(0:0.005:1, 0:0.005:1);
ylim([0 1]);
xlim([0 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Amplitude')
legend('Velocity Fiter', 'Ideal');
title('Velocity Estimator Frequency Response');

phase = 180 / pi * unwrap(angle(h_i));

subplot(3, 1, 2);
plot(w_i/pi, phase);
hold on;
plot([0, 1], [90, 90]);
legend('Velocity Fiter', 'Ideal');
%ylim(-1000, 90);
xlim([ 0, 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)');
ylabel('Phase Angle (degrees)');

subplot(3, 1, 3);
plot(w_i/pi, 100 * abs(abs(h_i) - (w_i/pi)) / (w_i/pi));
ylim([0 10]);
xlim([0 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Percent Error in Amplitude Resonse')


%% Acceleration filter
figure;
[b_lp_ii, a_lp_ii] = cheby2(accel_order, stopband_attenuation_accel_db, rel_cutoff_rad_accel);
freqz(b_lp_i, a_lp_i);
title('Low-Pass Filter for Acceleration');

% Filter on acceleration
filt_lp_ii = tf(b_lp_ii, a_lp_ii, Ts);

% differentiation and filtering for acceleration from position
diff_filter_ii = diff_i * filt_lp_i * diff_i * filt_lp_ii;
% Actual TF to work with to get acceleration from position
[b_filt_ii, a_filt_ii] = tfdata(diff_filter_ii);
% Unitless version for plotting
diff_filter_ii_ul = diff_i_ul * filt_lp_i * diff_i_ul * filt_lp_ii;
[b_lp_ii_ul, a_lp_ii_ul] = tfdata(diff_filter_ii_ul);

% differentiation and filtering for acceleration from velocity
diff_filter_ii_vel = diff_i * filt_lp_ii;
% Actual TF to work with to get acceleration from velocity
[b_filt_ii_vel, a_filt_ii_vel] = tfdata(diff_filter_ii_vel);
% Unitless version for plotting
diff_filter_ii_vel_ul = diff_i_ul * filt_lp_ii;
[b_lp_ii_vel_ul, a_lp_ii_vel_ul] = tfdata(diff_filter_ii_vel_ul);


%% Plot Acceleration Filter (Relative to Position)
figure;
subplot(3, 1, 1);
[h_ii, w_ii] = freqz(b_lp_ii_ul{1}, a_lp_ii_ul{1},'whole',2001);
plot(w_ii/pi, abs(h_ii));
hold on;
plot(0:0.005:1, (0:0.005:1).^2);

ylim([0 1]);
xlim([0 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Amplitude')
legend('Acceleration filter', 'Ideal');
title('Acceleration Estimator Frequency Response (relative to position signal)');

subplot(3, 1, 2);
plot(w_ii/pi, 180 / pi * unwrap(angle(h_ii)));
hold on;
plot([0, 1], [180, 180]);
xlim([0 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Phase Angle (degrees)')
legend('Acceleration filter', 'Ideal');

subplot(3, 1, 3);
norm_freqs = w_ii/pi;
plot(norm_freqs, 100 * abs(abs(h_ii) - norm_freqs.^2)./ norm_freqs.^2);
xlim([0 1]);
ylim([0, 25]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Percent Error in Amplitude Response')
legend('Acceleration Estimator');



%% Plot Acceleration Filter (Relative to Velocity)
figure;
subplot(3, 1, 1);
[h_vel_ii, w_vel_ii] = freqz(b_lp_ii_vel_ul{1}, a_lp_ii_vel_ul{1},'whole',2001);
plot(w_vel_ii/pi, abs(h_vel_ii));
hold on;
plot(0:0.005:1, (0:0.005:1));

ylim([0 1]);
xlim([0 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Magnitude')
legend('Acceleration Estimator', 'Ideal');
title('Acceleration Estimator Frequency Response (relative to velocity signal)');

subplot(3, 1, 2);
plot(w_vel_ii/pi, 180 / pi * unwrap(angle(h_vel_ii)));
hold on;
plot([0, 1], [90, 90]);
xlim([0 1]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Phase Angle (degrees)')
legend('Acceleration Estimator', 'Ideal');

subplot(3, 1, 3);
norm_freqs = w_vel_ii / pi;
plot(norm_freqs, 100 * abs(abs(h_vel_ii) - norm_freqs) ./ norm_freqs);
hold on;
xlim([0 1]);
ylim([0, 10]);
xticks(0:.5:2);
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Percent Error in Amplitude Response')
legend('Acceleration Estimator');

figure;
test_lowpass_twice_diff;
