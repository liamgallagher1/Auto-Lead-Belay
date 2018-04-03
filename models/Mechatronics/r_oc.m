clear;
% Calculates resistor to add to circuit to add overcurrent protection

i_oc_a = 4; % max current

R2 = 100000; % built in resistors
R3_old = 47000; % ohm

R3_new = R2 / (5 / 0.028 / i_oc_a - 1); % What we want here


R3_add = 1/(1/R3_new - 1/R3_old); % How we get it

%% What you actually made out of the resistors available
R3_add_actual = 2200;

R3_new_actual = 1 / (1 / R3_add_actual + 1 / R3_old); 

i_oc_actual = 5 / 0.028 * R3_new_actual / (R2 + R3_new_actual);

