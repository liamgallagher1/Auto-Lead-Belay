clear;
close all;

Ts = 0.001;

% Smoothes data, and selects a smoothing parameter. 
% Does two smoothings of the timestamped data, and compares the error
% between the acceleration estimates.

% Old avery test
%filename = '../../../pi/motor_driver/logs/timestamp_test39_17_12.csv';
%time_len = (1146444 / 12);
%A = csvread(filename, 2, 0, [2 0, 5, time_len -2]);

% Better test with true ISR
filename = '../../../pi/motor_driver/logs/timestamp_test48_17_48.csv';
A = csvread(filename, 2, 0);

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

% Relative to the full system
num_segments = 40;

% Params
%smoothness_params = [0.95, 0.99, 0.999];
%smoothness_params = [0.86, 0.87, 0.89, 0.9, 0.91, 0.92, 0.93];
%smoothness_params = [0.8, 0.82, 0.84, 0.85, 0.86, 0.87]; %, 0.89, 0.9, 0.91, 0.92, 0.93];
smoothness_params = 0.85;

%splits = [0.25, 0.3, 0.35, 0.45, 0.5];
splits  = 0.5;

% Against ground truth
SEs_counts = zeros(length(splits), length(smoothness_params));
AEs_counts = zeros(length(splits), length(smoothness_params));

% Velocity comparisons
SEs_vel = zeros(length(splits), length(smoothness_params));
AEs_vel = zeros(length(splits), length(smoothness_params));

% Acceleration comparison
SEs_accel = zeros(length(splits), length(smoothness_params));
AEs_accel = zeros(length(splits), length(smoothness_params));


for i = 1:length(splits)
    % TODO other loop here for more splits?
    
    % split evenly
    step = 1 / splits(i);
    to_include = step * (1:floor(length(times)*splits(i)));
    to_include = unique(round(to_include));
    to_not_include = setdiff(1:floor(length(times)), to_include);

    test_times       = times(to_include);
    test_counts      = counts(to_include);
    
    compare_times = times(to_not_include);
    compare_counts = counts(to_not_include);

    for j = 1:length(smoothness_params)        
        % Smooth both sets
        [coefs_test, start_indx_test, end_indx_test, breaks_test] = ...
            fast_smooth(test_times, test_counts, smoothness_params(j), round(num_segments * splits(i)));
        [coefs_compare, start_indx_compare, end_indx_compare, breaks_compare] = ...
            fast_smooth(compare_times, compare_counts, smoothness_params(j), round(num_segments * (1 - splits(i))));
        
        min_t = max(test_times(start_indx_test),  compare_times(start_indx_compare));
        max_t = min(test_times(end_indx_test),    compare_times(end_indx_compare));

        % Only look at test times in between
        sub_range = test_times > min_t & test_times < max_t;
        sub_validation_times  = test_times(sub_range);
        sub_validation_counts = test_counts(sub_range);
        discrete_times = min_t:Ts:max_t;

        % Make piecewise polynomial
        pp_test    = mkpp(breaks_test,     coefs_test);
        pp_compare = mkpp(breaks_compare,  coefs_compare);

        % Get count test for comparison
        count_test_est = ppval(pp_test, sub_validation_times);
        count_compare_est = ppval(pp_compare, sub_validation_times);
        error_diff = count_test_est - sub_validation_counts;
        sse_count = error_diff * error_diff';
        avg_error = 0.5 * (error_diff ./ count_test_est + error_diff ./sub_validation_counts);
        avg_error = sum(avg_error) / length(avg_error)
        SEs_counts(i, j) = sse_count;
        AEs_counts(i, j) = sum(abs(error_diff)) / length(error_diff);

        % Now get acceleration estimates. 
        % Go from 3rd order coefficents to 2nd order velocity
        first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
        % Go from 2nd order to 1st order acceleration 
        second_derv_mat = [2, 0; 0, 1; 0, 0];    
        
        % Calculate it for the test set
        first_derv_test = coefs_test * first_derv_mat;
        pp_test_vel = mkpp(breaks_test, first_derv_test);
        vel_est_test = ppval(pp_test_vel, discrete_times);
        
        second_derv_test = first_derv_test * second_derv_mat;
        pp_test_accel = mkpp(breaks_test, second_derv_test);
        accel_est_test = ppval(pp_test_accel, discrete_times);
        
        % Calculate it for the comparison set
        first_derv_compare = coefs_compare * first_derv_mat;
        pp_compare_vel = mkpp(breaks_compare, first_derv_compare);
        vel_est_compare = ppval(pp_compare_vel, discrete_times);
        
        second_derv_compare = first_derv_compare * second_derv_mat;
        pp_compare_accel = mkpp(breaks_compare, second_derv_compare);
        accel_est_compare = ppval(pp_compare_accel, discrete_times);
        
        % Compare the velocity estimates
        diff_vel = vel_est_test - vel_est_compare;
        avg_av_vel = sum(abs(diff_vel)) / length(diff_vel);
        avg_se_vel = (diff_vel * diff_vel') / length(diff_vel);
        SEs_vel(i, j) = avg_se_vel;
        AEs_vel(i, j) = avg_av_vel;
        
        % Compare the acceleration estimates
        diff_accel = accel_est_test - accel_est_compare;
        avg_av_accel = sum(abs(diff_accel)) / length(diff_accel);
        avg_se_accel = (diff_accel * diff_accel') / length(diff_accel);
        SEs_accel(i, j) = avg_se_accel;
        AEs_accel(i, j) = avg_av_accel;
        
        % Plot the things
        clf;
        subplot(3, 1, 1);
        plot(sub_validation_times, sub_validation_counts, 'gx');
        hold on;
        plot(sub_validation_times, count_compare_est);
        hold on;
        plot(sub_validation_times, count_test_est);
        legend('Truth', 'Compare Set Estimate', 'Test Set Estimate');
        
        subplot(3, 1, 2);
        plot(discrete_times, vel_est_test);
        hold on;
        plot(discrete_times, vel_est_compare);
        legend('Test Set Estimate', 'Comapare Set Estimate');
        
        subplot(3, 1, 3);
        plot(discrete_times, accel_est_test);
        hold on;
        plot(discrete_times, accel_est_compare); 
        legend('Test Set Estimate', 'Compare Set Estimate');
        
    end
end

save('smoothness_accel_cv_5.mat', 'SEs_counts', 'AEs_counts', 'SEs_vel', 'AEs_vel', 'SEs_accel', 'AEs_accel', 'splits', 'smoothness_params');



figure;
subplot(2, 1, 1);
for i = 1:length(smoothness_params) 
    loglog(splits, SEs_accel(:, i), '-.');
    hold on;
end
ylabel('Average sum of squares error');
xlabel('Log of split percentage');
legend('0.86', '0.87', '0.89', '0.9', '0.91', '0.92', '0.93');

legend('0.65', '0.75', '0.8', '0.9', '0.95', '0.98');
%smoothness_params = ['0.8', '0.82', '0.84', '0.85', '0.86, 0.87];
% Whatever, about 0.85 seems great

subplot(2, 1, 2);
for i = 1:length(smoothness_params) 
    loglog(splits, AEs_accel(:, i), '-.');
    hold on;
end
ylabel('Average absolute value error');
xlabel('Log of split percentage');
legend('0.86', '0.87', '0.89', '0.9', '0.91', '0.92', '0.93');
%legend('0.65', '0.75', '0.8', '0.9', '0.95', '0.98');








