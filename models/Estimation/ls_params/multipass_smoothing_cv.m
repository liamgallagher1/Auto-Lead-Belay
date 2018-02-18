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

%num_segments = 40;
num_segments = 20;
num_folds = length(times);

in_test  = (num_folds - 1) * floor(length(times) / num_folds);

% middle was best
%smoothness_params = [0.0001, 0.0003, 0.001, 0.003, 0.01];
%0.9 was way better than i've seen
%smoothness_params = [0.03, .1, .3, .9];
%.99 best by a bit
%smoothness_params = [0.8, 0.9, 0.95, 0.99];
%0.995 best
%smoothness_params = [0.98, 0.99, 0.995, 0.999, 0.9995];
%.9999 best
%smoothness_params = [0.999, 0.9993, 0.9999, 0.99993, 0.99999, 0.999993];
%smoothness_params = logspace(log(0.99999), log(0.9999999), 10);
% therefore
smoothness_params = 0.9999947;

SEs = zeros(length(smoothness_params), num_folds);
ABs = zeros(length(smoothness_params), num_folds);



for i = 1:length(smoothness_params)
    % randomly order
    rand_order = randperm(length(times));
    times = times(rand_order);
    counts = counts(rand_order);
    folds = mod([1:length(times)], num_folds) + 1; 
    for j = 1:num_folds
        test_times = times(folds ~= j);
        test_counts = counts(folds ~= j);
        validation_times = times(folds == j);
        validation_counts = counts(folds == j);

        % Resort
        [test_times, indx] = sort(test_times);
        test_counts = test_counts(indx);
        [validation_times, indx] = sort(validation_times);
        validation_counts = validation_counts(indx);

        [coefs, start_indx, end_indx, breaks] = ...
            fast_smooth(test_times, test_counts, smoothness_params(i), num_segments);
        min_t = test_times(start_indx);
        max_t = test_times(end_indx);
        % Only look at test times in between
        sub_range = validation_times > min_t & validation_times < max_t;
        sub_validation_times = validation_times(sub_range);
        sub_validation_counts = validation_counts(sub_range);

        % Make piecewise polynomial
        pp = mkpp(breaks, coefs);

        count_est = ppval(pp, sub_validation_times);
        
        clf;
        subplot(3, 1, 1);
        plot(sub_validation_times, count_est);
        hold on;
        plot(sub_validation_times, sub_validation_counts);
        legend('Est', 'Truth');

        error_diff = count_est - sub_validation_counts;
        sse = error_diff * error_diff';
        ave_se = sse / length(count_est);
        ave_e = sum(abs(error_diff)) / length(count_est);
        SEs(i, j) = ave_se;
        ABs(i, j) = ave_e; 

        % Go from 3rd order coefficents to 2nd order velocity
        first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
        % Go from 2nd order to 1st order acceleration 
        second_derv_mat = [2, 0; 0, 1; 0, 0];

        first_derv = coefs * first_derv_mat;
        pp = mkpp(breaks, first_derv);
        vel_est = ppval(pp, sub_validation_times);
        
        second_derv = first_derv * second_derv_mat;
        pp = mkpp(breaks, second_derv);
        accel_est = ppval(pp, sub_validation_times);
        
        subplot(3, 1, 2);
        plot(sub_validation_times, vel_est);
        subplot(3, 1, 3);
        plot(sub_validation_times, accel_est);
        
        
    end
end


save('smoothness_cv2.mat', 'SEs', 'ABs', 'smoothness_params');







