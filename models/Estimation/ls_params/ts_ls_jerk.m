clear;
close all;
clf;

omega_weight = 0; % 100; %100;
alpha_weight = 0; % 0.1;
jerk_weight =  5 * 10^-5; % 5 * 10^-15; %5 * 10^-35;

% Load 'time', 'theta_t', 'omega_t', 'alpha_t'
load('x_v_a_guess.mat');
% Load time_stamps, counts
load('time_stamps_avery.mat');

% Make the time start from zero seconds
time_stamps = time_stamps - time(1);
counts = counts(mod(1:length(time_stamps), 5) == 0);
time_stamps = time_stamps(mod(1:length(time_stamps), 5) == 0);

time = time - time(1);
Ts = time(2) - time(1);

% Filtered estimates
theta_fil = zeros(length(time), 1);
omega_fil = zeros(length(time), 1);
alpha_fil = zeros(length(time), 1);

sses = zeros(length(time), 1);
ses_omega = zeros(length(time), 1);
ses_alpha = zeros(length(time), 1);
ses_jerk  = zeros(length(time), 1); 

order = 5;
num_counts = 40;
sigma = 1;


% Do a simple test. 
start_indx = num_counts * sigma + 1;

% Used to avoid doing find over whole sorted array
prev_indx = 1;

for i = start_indx:1:length(time)
    curr_time = time(i);
    % Find the last time stamp less than the curent time
    stamp_indx = find(time_stamps(prev_indx:end) > curr_time, 1) - 1  + prev_indx - 1;
    stamp_buffer = time_stamps(stamp_indx - start_indx : stamp_indx);
    count_buffer = counts(stamp_indx - start_indx : stamp_indx);
    
    
    omega_prior = 0; %omega_fil(i - 1);% + Ts * alpha_fil(i - 1);
    if (isnan(omega_prior) || isinf(omega_prior))
        omega_prior = 0;
    end
    alpha_prior = 0; %alpha_fil(i - 1);
   
%     [theta_hat, omega_hat, alpha_hat, sse_pos, se_omega, se_alpha] =...
%         ts_guess_lp(count_buffer, stamp_buffer, sigma, num_counts, order, ...
%                     curr_time, omega_prior, omega_weight, alpha_prior, alpha_weight);

    
    [theta_hat, omega_hat, alpha_hat, sse_pos, se_omega, se_alpha, se_jerk] =...
        ts_guess_lp_jerk(count_buffer, stamp_buffer, sigma, num_counts, order, ...
                    curr_time, omega_prior, omega_weight, alpha_prior, alpha_weight, jerk_weight);
                
    theta_fil(i) = theta_hat;
    omega_fil(i) = omega_hat;
    alpha_fil(i) = alpha_hat;
    sses(i) = sse_pos;
    ses_omega(i) = se_omega;
    ses_alpha(i) = se_alpha;
    ses_jerk(i) = se_jerk;

    prev_indx = stamp_indx;
end

c_t_r = 2 * pi / (4 * 2048); 

time = time(start_indx:end) - time(start_indx);
theta_t = theta_t(start_indx:end); % * c_t_r;
theta_fil = theta_fil(start_indx:end) * c_t_r;
omega_fil = omega_fil(start_indx:end) * c_t_r;
omega_t = omega_t(start_indx:end);% * c_t_r;
alpha_fil = alpha_fil(start_indx:end) * c_t_r;
alpha_t = alpha_t(start_indx:end);% * c_t_r;
sses = sses(start_indx:end);
ses_omega = ses_omega(start_indx:end);
ses_alpha = ses_alpha(start_indx:end);
ses_jerk = ses_jerk(start_indx:end);


% Plot results
% x_range = [22.3, 23];
x_range = [min(time_stamps), max(time_stamps)];

subplot(3, 1, 1); 
plot(time_stamps, counts * c_t_r, 'gx'); 
hold on;
plot(time, theta_fil);
hold on;
plot(time, theta_t);
ylim([0, 1.5 * max(theta_t)]);
xlim(x_range); 
xlabel('Time [s]');
ylabel('Theta [rad]');
legend('Stamps', 'Theta Filtered', 'Theta Smoothed');

subplot(3, 1, 2); 
plot(time, omega_fil);
hold on;
plot(time, omega_t);
ylim([-2 * max(omega_t), 2 * max(omega_t)]);
xlim(x_range); 

xlabel('Time [s]');
ylabel('Omega [rad/s]');
legend('Omega Filtered', 'Omega Smoothed');

subplot(3, 1, 3); 
plot(time, alpha_fil);
hold on;
plot(time, alpha_t);
ylim([-2 * max(alpha_t), 2 * max(alpha_t)]);
xlim(x_range); 

xlabel('Time [s]')
ylabel('Alpha [rad/s^2]'); 
legend('Alpha Filtered', 'Alpha Smoothed');

figure;
subplot(4, 1, 1);
plot(time, sses);
%ylim([0, 20]);
xlabel('Time [s]')
ylabel('Sum of squared error [count^2]'); 

subplot(4, 1, 2);
plot(time, ses_omega);
%ylim([0, 5 * 10^5]);
xlabel('Time [s]')
ylabel('Sum of squared velocity error [rad^2/s^2]');

subplot(4, 1, 3);
plot(time, ses_alpha);
%ylim([0, 5 * 10^7]);
xlabel('Time [s]')
ylabel('Sum of squared acceleration error [rad^2/s^4]'); 

subplot(4, 1, 4);
plot(time, ses_jerk);
%ylim([0, 5 * 10^14]);
xlabel('Time [s]')
ylabel('Sum of squared acceleration error [rad^2/s^4]'); 








