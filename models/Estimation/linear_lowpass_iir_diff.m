clear;
close all;

% http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=4063529
Ts = 0.001;
sample_freq_hz = 1 / Ts;

% Want it high but not so high that the signals get too noisy
passband_freq_hz = 100;

rel_cutoff_rad = 2 * passband_freq_hz / sample_freq_hz; 

order = 4;
stopband_attenuation_db = 100; 
stopband_ripple_db = 0.1;

% Lowpass Chebyshev Type I
% [b_lp, a_lp] = cheby1(order, stopband_ripple_db, rel_cutoff_rad);
% freqz(b_lp, a_lp);

% Nah, use Type 2
[b_lp, a_lp] = cheby2(order, stopband_attenuation_db, rel_cutoff_rad);
freqz(b_lp, a_lp);

filt_lp = tf(b_lp, a_lp, Ts);

% First order wide band diff: Al-Alaoui operator
% appropriate version
diff_i = 8 / 7 / Ts * tf([1, -1], [1, 1/7], Ts);
% unitless version or something
diff_i_ul = 0.3638 * tf([ 1, -1], [1, 1 / 7], Ts);
diff_ii_ul = 0.25586 * tf([1, 0, -1], [1, 0.5358, 0.0718], Ts);

diff_filter_i = diff_i * filt_lp;

% Unitless version for plotting
diff_filter_i_ul = diff_i_ul * filt_lp;
diff_filter_ii_ul = diff_ii_ul * filt_lp;

% Differentiator for comparison
diff_c = tf([1, 0], 1);
diff_d = c2d(diff_c, Ts, 'tustin');
[b_diff, a_diff] = tfdata(diff_d);

[b_filt_i_ul, a_filt_i_ul] = tfdata(diff_filter_i_ul);
[b_filt_ii_ul, a_filt_ii_ul] = tfdata(diff_filter_ii_ul);
freqz(b_filt_i_ul{1}, a_filt_i_ul{1});
freqz(b_filt_ii_ul{1}, a_filt_ii_ul{1});

[h_i,w_i] = freqz(b_filt_i_ul{1}, a_filt_i_ul{1},'whole',2001);
[h_ii,w_ii] = freqz(b_filt_ii_ul{1}, a_filt_ii_ul{1},'whole',2001);

plot(w_i/pi, abs(h_i));
hold on;
plot(w_ii/pi, abs(h_ii));
hold on;

plot([0, 1], [0, 1]);
ax = gca;
ax.YLim = [0 1];
ax.XLim = [0 1];
ax.XTick = 0:.5:2;
xlabel('Normalized Frequency (\times\pi rad/sample)')
ylabel('Magnitude (dB)')
legend('Diff Type 1', 'Diff Type 2', 'Ideal');

%hfvt = fvtool(b_filt_i{1}, a_filt_i{1}, b_filt_ii{1}, a_filt_ii{1});

% Actual TF to work with to get velocity
[b_filt_i, a_filt_i] = tfdata(diff_filter_i);








