clear;

a_22 = 0;
a_23 = 0;
b_2 = 0;
c_22 = 0;
d_2 = 0;
parameters = {'a_22', a_22; 'b_2', b_2; 'c_22', c_22; 'd_2', d_2};
odefun = 'upper_pulley_dynamics';
fcn_type = 'c';
init_sys = idgrey(odefun, parameters, fcn_type);


%% Load data
% M = csvread('logs/upper_pulley_id_3_93_1_22.csv', 1, 0);
M = csvread('logs/test_rope_flaked_93_18_40.csv', 1, 0);

sm_current = M(:, 12); % Noisy estimate of the current
T = length(sm_current);

% sm_current = sm_current - mean(sm_current); % mean center
% sm_current = sm_current - sign(sm_current) .*...
%     min(abs(sm_current), i_friction * ones(T, 1));


sm_count   = M(:, 14); % Encoder counts
sm_v = 12 * M(:, 18); % signed, in volts
Ts = 0.001;
input = sm_v;

output = [sm_current, sm_count];

data = iddata(output, input, Ts);

%% Estimate system

% % Two dof system
est_sys_1 = greyest(data, init_sys);
opt = compareOptions('InitialCondition', 'zero');
compare(data, est_sys_1, Inf, opt);


%% Term for friction
odefun = 'upper_pulley_dynamics_expanded';
fcn_type = 'c';
params = getpvec(est_sys_1);
a_22 = params(1);
b_2 = params(2);
c_22 = params(3);
d_2 = params(4);
a_23 = 0;

parameters = {'a_22', a_22; 'a_23', a_23; 'b_2', b_2; 'c_22', c_22; 'd_2', d_2};
init_sys = idgrey(odefun, parameters, fcn_type);


% Two dof system
%opt = greyestOptions('EnforceStability', true);

est_sys_2 = greyest(data, init_sys);
opt = compareOptions('InitialCondition', 'zero');
compare(data, est_sys_2, Inf, opt);



