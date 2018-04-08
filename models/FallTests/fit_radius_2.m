clear;

%% Lets fit a radius


%%
M = csvread('logs/sys_test94_3_43.csv', 1, 0);
%M = csvread('logs/no_drive_log_test_96_16_1.csv', 1, 0);
rope_length = 60;

time = M(:, 1) + 10^-9 * M(:, 2);
time = time - time(1);
up_counts = -1 * M(:, 13);
spool_counts = 1 * M(:, 14);

CPR = 8196;
up = up_counts * 2 * pi / CPR;
spool = spool_counts * 2 * pi / CPR; 

%up_radius = rope_length / max(up); % 6.02 cm radius
up_radius = 0.027966354424642;

rope_out = up_radius * up;
avg_spool_radius = rope_length / max(spool); %0.1923


subplot(2, 1, 1);
plot(rope_out, spool);
xlabel('Rope Out [m]');
ylabel('Spool [rad]');
title('60m rope pull through')


%% Figure out radii

indx = up < 900;
time = time(indx);
up = up(indx);
spool = spool(indx);
rope_out = rope_out(indx);
spool_fit = fit(spool, rope_out,'smoothingspline');

coefs = spool_fit.p.coefs();
breaks = spool_fit.p.breaks();
first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
first_derv_coefs = coefs * first_derv_mat;
% d rope_out / d spool 
first_derv_fit = mkpp(breaks, first_derv_coefs);
radius = ppval(time, first_derv_fit);

figure;
plot(rope_out, radius);
hold on
radius_fit = robustfit(up, radius);
% radius_fit = %[0.205860837036968, -0.000006439609330]
aprox_radius = radius_fit(1) + radius_fit(2) * up;

plot(rope_out, aprox_radius);
xlabel('Rope Out [m]');
ylabel('Spool Radius [m]');
legend('Observed', 'Fit');
ylim([0, 0.25]);

%% Test solution
r0 = radius_fit(1);
a = radius_fit(2); %* r0;


s_theta = r0 / a - r0 /a * exp(-a * spool);
figure;
plot(time, rope_out);
hold on;
plot(time, s_theta);
xlabel('Time');
ylabel('Rope Out [m]');
legend('Ground Truth', 'Model');




% r0 =
%   0.185789405425864
%a =
%    -5.492535985428214e-06

% r0 = 0.091241941472015
% a = -9.645565676206025e-06
% up_radius = 0.027966354424642

