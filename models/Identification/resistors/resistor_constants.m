%clear;
close all;

%Time s	time ns	loop count	la_raw_adc	la_raw_adc	la_amp_adc	sm_raw_adc	sm_amp_adc	lm_raw_adc	lm_amp_adc	la_current	sm_current	lm_current	lm_count	sm_count	sm_pos_r	sm_vel_est_rs	lm_pos_r	sm_vel_est_rs
%M = csvread('logs/first_resistor_val_test92_1_2.csv', 1, 0);
%M = csvread('logs/res_test_2_92_11_23.csv', 1, 0);
%M = csvread('logs/res_test_3_92_11_39.csv', 1, 0);
%M = csvread('logs/res_test_4_92_11_51.csv', 1, 0);
%M = csvread('logs/res_test_bm_volt92_12_2.csv', 1, 0);
%M = csvread('logs/res_test_la_volt92_12_4.csv', 1, 0);
%M = csvread('logs/rest_test_driven_1_92_13_16.csv', 1, 0);
%M = csvread('logs/soft_r_test92_14_33.csv', 1, 0);

% Better tests using a power suply
%M = csvread('logs/res_test_ps_bm__92_14_47.csv', 1, 0);
%M = csvread('logs/res_test_ps_sm_92_14_48.csv', 1, 0);
%M = csvread('logs/res_test_ps_la_92_14_46.csv', 1, 0);

M = csvread('logs/no_draw_bm_92_15_8.csv', 1, 0);



la_raw_amp = M(:, 4:5);
sm_raw_amp = M(:, 6:7);
lm_raw_amp = M(:, 8:9);
T = length(la_raw_amp(:, 1));

% averages from lm_raw
mean_raw = mean(lm_raw_amp(:, 1));
mean_amp = mean(lm_raw_amp(:, 2));


%% Get resistor values
la_amp = robustfit(la_raw_amp(:, 1), la_raw_amp(:, 2), 'bisquare', 4.685, 'off');
%4.5458

sm_amp = robustfit(sm_raw_amp(:, 1), sm_raw_amp(:, 2), 'bisquare', 4.685, 'off');
% 3.2912

lm_amp = robustfit(lm_raw_amp(:, 1), lm_raw_amp(:, 2));
% lm_amp =1.0e+03 * [-5.9018, 0.0034];
lm_a_vcc = -3.3 / 4095 * lm_amp(1) / lm_amp(2);
% 1.4119

% Nominal Resistor Values
BOARD_2_R1 = 100000.0; 
BOARD_2_R2 = 220000.0;

BOARD_4_R6 = 100000.0;
BOARD_4_R7 = 330000.0;
BOARD_4_R5 = 470000.0;
BOARD_4_R4 = 620000.0;

BOARD_4_R1 = 100000.0;
BOARD_4_R3 = 330000.0;

la_expected_amp = 1 + BOARD_4_R7/BOARD_4_R6;
sm_expected_amp = 1 + BOARD_2_R2 / BOARD_2_R1;
a_vcc_d = BOARD_4_R5 / (BOARD_4_R5 + BOARD_4_R4) * 4095;
lm_expected_amp = BOARD_4_R3 / BOARD_4_R1 * [-a_vcc_d; 1];




subplot(3, 1, 1);
plot(la_raw_amp(:, 1), la_raw_amp(:, 2), 'rx');
hold on;
plot(la_raw_amp(:, 1), la_amp * la_raw_amp(:, 1), 'go');
hold on;
plot(la_raw_amp(:, 1), la_expected_amp * la_raw_amp(:, 1), 'b');
legend('Amplified measurements', 'Fit amplified estimates', 'Expected amplified estimates');


subplot(3, 1, 2);
plot(sm_raw_amp(:, 1), sm_raw_amp(:, 2), 'rx');
hold on;
plot(sm_raw_amp(:, 1), sm_amp * sm_raw_amp(:, 1), 'go');
hold on;
plot(sm_raw_amp(:, 1), sm_expected_amp * sm_raw_amp(:, 1), 'b');


subplot(3, 1, 3);
plot(lm_raw_amp(:, 1), lm_raw_amp(:, 2), 'rx');
hold on;
ests = [ones(T, 1), lm_raw_amp(:, 1)] * lm_amp;
plot(lm_raw_amp(:, 1), ests, 'go');
hold on;
ests = [ones(T, 1), lm_raw_amp(:, 1)] * lm_expected_amp;
plot(lm_raw_amp(:, 1), ests, 'b');



figure;
subplot(3, 1, 1);
yyaxis left
plot(1:T, la_raw_amp(:, 1));
yyaxis right
plot(1:T, la_raw_amp(:, 2));

subplot(3, 1, 2);
yyaxis left
plot(1:T, sm_raw_amp(:, 1));
yyaxis right
plot(1:T, sm_raw_amp(:, 2));

subplot(3, 1, 3);
yyaxis left
plot(1:T, lm_raw_amp(:, 1));
yyaxis right
plot(1:T, lm_raw_amp(:, 2));

