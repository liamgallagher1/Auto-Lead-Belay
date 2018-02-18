function [theta_hat, omega_hat, alpha_hat, sse] =...
    ts_guess(counts, times, sigma, num_counts, order, curr_time)
% counts : vector of encoder counts, the end of which is the most recently
% observed one
% times : a vector of corresponding times [s] to the counts
% sigma : skip param, use every sigma counts. 
% num_counts : number of counts to use (assume length(times) > num_counts *
% sigma
% order : order of polynomial to fit
% Fits a LS estimate through the past number of counts. 
% Based on: Velocity and acceleration estimation for optical 
% incremental encoders
% by R.J.E. Merry *, M.J.G. van de Molengraft, M. Steinbuch

% Outputs in counts, counts / s, counts / s^2


% Counts and times to use for this fit, reverse order
use_indx = length(counts):...
            -1 * sigma:...
            length(counts) - (num_counts - 1) * sigma;        

% Now in reverse order
sub_c = counts(use_indx);
sub_t = times(use_indx);

% % Possible add a phantom count if its been a while since we've gotten
% % a reading
% since_last = curr_time - sub_t(1);
% theta_prior = sub_c(1) + vel_prior * since_last + 0.5 * accel_prior * since_last^2;
% if (abs(theta_prior - suc_c(1)) > 1)
%    phantom_c = sub_c + sign(theta_prior - suc_c(1));
%    sub_c = [phantom_c, sub_c];
%    sub_t = [curr_time, sub_t];
% end


% Normalize time
delta_t = sub_t(1) - sub_t(end);
norm_t = (sub_t - sub_t(end)) ./ delta_t;
norm_c = sub_c - sub_c(end);

% Set up problem matrix
A = zeros(num_counts, order + 1);
B = norm_c';

% Vectorization could have been fun
for k = 1:num_counts
    for pow = 0:order
        A(k, order + 1 - pow) = norm_t(k)^pow;
    end
end

% Solve LS problem
P = A \ B; %

% Assemble derivative matrix (every damn time!)
first_derv = eye(order + 1);
second_derv = eye(order);
for pow = 0:order
    first_derv(order + 1 - pow, order + 1 - pow) = pow;
end
for pow = 0:order-1
    second_derv(order - pow, order - pow) = pow;
end

% Use them to differentiate the polynomial
P_v = first_derv * P;
P_v = P_v(1:end-1);
% And again !
P_a = second_derv * P_v;
P_a = P_a(1:end - 1);

% Normalize the current time
curr_time_n = (curr_time - sub_t(end)) / delta_t;
% Vector of exponentiated powers
curr_time_p = zeros(1, order + 1);
for pow = 0:order
    curr_time_p(order + 1 - pow) = curr_time_n^pow;
end

% Estimate the current position, velocity, acceleration
theta_hat = curr_time_p * P;
% Correct for count "normalization"
theta_hat = theta_hat + sub_c(end);

curr_time_p = curr_time_p(2:end);
omega_hat = curr_time_p * P_v;
% Correct for time dilation
omega_hat = omega_hat / delta_t;

curr_time_p = curr_time_p(2:end);
alpha_hat = curr_time_p * P_a;
% Twice correct for time dilation
alpha_hat = alpha_hat / delta_t^2;

error = A * P - B;
sse = error' * error;

% %% plotting code to debug
% 
% num_plot_points = 100;
% % Times in seconds
% time_plot_s = sub_t(end): (curr_time - sub_t(end)) / num_plot_points : curr_time;
% % Normalized times
% time_plot_n = (time_plot_s - sub_t(end)) / delta_t;
% % Normalized times raised to powers
% time_plot_p = zeros(length(time_plot_s), order + 1);
% 
% for pow = 0:order
%     time_plot_p(:, order + 1 - pow) = time_plot_n.^pow';
% end
% 
% 
% theta_hat_t = time_plot_p * P + sub_c(end); 
% vel_hat_t = time_plot_p(:, 2:end) * P_v;
% vel_hat_t = vel_hat_t / delta_t;
% alpha_hat_t = time_plot_p(:, 3:end) * P_a;
% alpha_hat_t = alpha_hat_t / delta_t^2;
% 
% % counts to radians
% c_t_r = 2 * pi / (4 * 2048);
% clf;
% subplot(3, 1, 1);
% plot(time_plot_s, theta_hat_t * c_t_r);
% hold on; 
% plot(sub_t, sub_c * c_t_r, 'x');
% xlabel('Time [s]');
% ylabel('Radial position [rad]');
% 
% subplot(3, 1, 2);
% plot(time_plot_s, vel_hat_t * c_t_r);
% xlabel('Time [s]');
% ylabel('Radial velocity [rad/s]');
% 
% 
% subplot(3, 1, 3); 
% plot(time_plot_s, alpha_hat_t * c_t_r);
% xlabel('Time [s]');
% ylabel('Radial acceleration [rad/s^2]');


end

