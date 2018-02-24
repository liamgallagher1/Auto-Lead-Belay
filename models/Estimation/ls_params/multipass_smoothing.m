clear;

filename = '../../../pi/motor_driver/logs/timestamp_test39_17_12.csv';

time_len = (1146444 / 12);

A = csvread(filename, 2, 0, [2 0, 5, time_len -2]);
% pull time stamps
stamps_s = A(1, 1:end-1);
stamps_ns = A(2, 1:end-1); 
counts = -1 * A(3, 1:end-1);
times = stamps_s - stamps_s(1) + 10^-9 * (stamps_ns - stamps_ns(1));
% Clean data by removing all duplicates, however they happened.
time_diff = diff(times);
times_to_keep = time_diff ~= 0;
times = times(times_to_keep);
counts = counts(times_to_keep);

times =   times(1:10000);
counts = counts(1:10000);

[coefs, start_indx, end_indx, breaks] = ...
    fast_smooth(times, counts, 0.95, 30); 
% Go from 3rd order coefficents to 2nd order velocity
first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
% Go from 2nd order to 1st order acceleration 
second_derv_mat = [2, 0; 0, 1; 0, 0];

first_derv = coefs * first_derv_mat;
second_derv = first_derv * second_derv_mat;









