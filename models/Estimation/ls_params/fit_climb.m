clear;

% Fits a smoothing spline to timestamp data

filename = '../../../pi/motor_driver/logs/timestamp_test39_17_12.csv';

time_len = (1146444 / 12);

A = csvread(filename, 2, 0, [2 0, 5, time_len -2]);
% pull time stamps
stamps_s = A(1, 1:end-1);
stamps_ns = A(2, 1:end-1); 
counts = -1 * A(3, 1:end-1);
times = stamps_s - stamps_s(1) + 10^-9 * (stamps_ns - stamps_ns(1));

% Interpolate at 1000 hz
steps = 0:0.001:times(end);
[unique_counts, count_indx] = unique(counts);
counts_interp = interp1(unique_counts, times(count_indx), steps);

% y = fft(counts_interp);                               % Compute DFT of x
% m = abs(y);                               % Magnitude
% p = unwrap(angle(y));                     % Phase
% f = (0:length(y)-1)*100/length(y);
% subplot(2,1,1)
% plot(f,m)
% title('Magnitude')
% ax = gca;
% ax.XTick = [15 40 60 85];
% subplot(2,1,2)
% plot(f,p*180/pi)
% title('Phase')
% ax = gca;
% ax.XTick = [15 40 60 85];

%plot(times, counts);

% Naw, stay with smoothing splines Try SGOlay filtering

% smooth_counts = smooth(times, counts,span,'sgolay',degree);


