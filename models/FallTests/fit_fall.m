clear;
close all;
% Set these four fall specific parameters
t_start = 3.5;
t_stop = 12;
inital_rope_out = 25; % aprox
inital_height = 15.24;

% Read data
% M = csvread('logs/test_10_100_pounds_94_14_42.csv', 1, 0);
% M = csvread('logs/test_15_liam_real_final_94_15_41.csv', 1, 0);
M = csvread('logs/test_18_60_1_floor_94_16_41.csv', 1, 0);


spool_counts = 1 * M(:, 14);
time = M(:, 1) + 10^-9 * M(:, 2);
time = time - time(1);

% Convert to fall height over time
T = length(spool_counts);
CPR = 8196;
spool_rad = spool_counts * 2 * pi / CPR;
fall_heights = fall_length(inital_rope_out, inital_height, spool_rad);
% Plot to find t_start and t_stop
plot(time, fall_heights);

% Clean the data to that range
indx = time > t_start  & time < t_stop;
time = time(indx);
time = time - time(1);
fall_heights = fall_heights(indx);
fall_heights = fall_heights - fall_heights(end);

% Make Better graph



% Fit velocity data
height_fit = fit(time, fall_heights,'smoothingspline');
coefs = height_fit.p.coefs();
breaks = height_fit.p.breaks();
% Differentiate it
first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
first_derv_coefs = coefs * first_derv_mat;
first_derv_fit = mkpp(breaks, first_derv_coefs);
fall_vels = ppval(time, first_derv_fit);
% Differentiate it again
vel_fit = fit(time, fall_vels,'smoothingspline', 'SmoothingParam', 0.9995);
coefs = vel_fit.p.coefs();
breaks = vel_fit.p.breaks();
second_derv_coefs = coefs * first_derv_mat;
second_derv_fit = mkpp(breaks, second_derv_coefs);
fall_accels = ppval(time, second_derv_fit);



clf;
subplot(3, 1, 1);
plot(time, fall_heights);
xlabel('Time [s]');
ylabel('Height [m]');

subplot(3, 1, 2);
plot(time, fall_vels);
xlabel('Time [s]');
ylabel('Velocity [m/s]');

subplot(3, 1, 3);
plot(time, fall_accels);
xlabel('Time [s]');
ylabel('Acceleration [m/s/s]');





