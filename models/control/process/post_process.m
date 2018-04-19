clear;
clf;

%%
%M = csvread('logs/told_grahm_25_cm_96_18_51.csv', 1, 0);
%M = csvread('logs/test_error_out_97_11_34.csv', 1, 0);
M = csvread('logs/test_error_out_97_11_36.csv', 1, 0);


rope_length = 60;

time = M(:, 1) + 10^-9 * M(:, 2);
time = time - time(1);
up_counts = -1 * M(:, 13);
spool_counts = 1 * M(:, 14);

up_theta = M(:, 15);
spool_theta = M(:, 17);
up_vel = M(:, 16);
spool_vel = M(:, 18);
error_rs =  M(:, 25);
error_rad = M(:, 24);
slack_out = M(:, 23);
rope_out = M(:, 22);
dc = M(:, 21);




%% Plots
yyaxis left
plot(time, slack_out);
hold on;
plot([0, max(time)], [0.5, 0.5]);

yyaxis right
plot(time, rope_out); 
legend('Estimated Slack Out', 'Target Slack', 'Total Rope Feed Out');


yyaxis left
title('Slack Control');
xlabel('Time [s]');
ylabel('System Slack [m]');

yyaxis right
ylabel('Rope Out [m]');


% figure;
% subplot(3, 1, 1);
% plot(time, rope_out);
% yyaxis right;
% plot(time, slack_out);
% xlabel('Time [s]');
% ylabel('Theta [s]');
% legend('Rope Out', 'Slack Out');
% subplot(3, 1, 2);
% plot(time, 12 * dc);
% xlabel('Time [s]');
% ylabel('Voltage [V]');
% subplot(3, 1, 3);
% plot(time, error_rad);
% yyaxis right;
% plot(time, error_rs);
% 
% 
% 
% 
% 
% 
% %% ID data
% 
% %data = iddata(spool_theta, up_theta, 0.0005);
% D = finddelay(up_theta, spool_theta);



