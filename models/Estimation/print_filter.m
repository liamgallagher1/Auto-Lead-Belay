function print_filter(b_filt, a_filt)
%PRINT_FILTER prints it so that I can copy it
format long

b_string = sprintf('%16.16f, ' , b_filt);
b_string = b_string(1:end-2);% strip final comma
b_string = ['static double vel_estimator_b[VEL_ESTIMATOR_ORDER] = {',...
    b_string, '};'];
disp(b_string)

a_string = sprintf('%16.16f, ' , a_filt);
a_string = a_string(1:end-2);% strip final comma
a_string = ['static double vel_estimator_a[VEL_ESTIMATOR_ORDER] = {',...
    a_string, '};'];
disp(a_string)





end

