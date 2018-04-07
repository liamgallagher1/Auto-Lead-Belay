clear;

%% Lets fit a radius


%%
M = csvread('logs/sys_test94_3_43.csv', 1, 0);
time = M(:, 1) + 10^-9 * M(:, 2);
time = time - time(1);
up_counts = -1 * M(:, 13);
spool_counts = M(:, 14);

CPR = 8192;
up = up_counts * 2 * pi / CPR;
spool = spool_counts * 2 * pi / CPR; 

subplot(2, 1, 1);
plot(up, spool);
xlabel('Upper Pulley [rad]');
ylabel('Spool [rad]');
title('60m rope pull through')
subplot(2, 1, 2);
plot(time, [up, spool]);
legend('Upper Pulley', 'Spool');
xlabel('Time [s]');


%% Figure out radii
up_radius = 60 / max(up); % 6.02 cm radius
avg_spool_radius = 60 / max(spool); %0.1923

indx = up < 900;
time = time(indx);
up = up(indx);
spool = spool(indx);

spool_fit = fit(up, spool,'smoothingspline');

coefs = spool_fit.p.coefs();
breaks = spool_fit.p.breaks();
first_derv_mat = [3, 0, 0; 0, 2, 0; 0, 0, 1; 0, 0, 0];
first_derv_coefs = coefs * first_derv_mat;
% d spool / d up
first_derv_fit = mkpp(breaks, first_derv_coefs);
ratio = ppval(time, first_derv_fit);

figure;
rope_out = up_radius * up;
plot(rope_out, up_radius ./ ratio);
hold on
ratio_fit = robustfit(up, ratio);
% ratio =    [0.280977082100127, 128796508961646e-05]
aprox = ratio(1) + up * ratio(2);
plot(rope_out, up_radius ./ aprox);
xlabel('Rope Out [m]');
ylabel('Spool Radius [m]');
legend('Observed', 'Fit');
figure;
rope_out = up_radius * up;
plot(rope_out, ratio);
hold on
ratio_fit = robustfit(up, ratio);
% ratio =    [0.280977082100127, 128796508961646e-05]
aprox = ratio_fit(1) + up * ratio_fit(2);
plot(rope_out, aprox);
xlabel('Rope Out [m]');
ylabel('Spool Radius [m]');
legend('Observed', 'Fit');

figure;




