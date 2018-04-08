clear;
%clean_sys_id;
%load('logs/one_dir_data.mat');
%load('logs/harder_faster_sigmoid.mat');
load('logs/more_filtered_sigmoid.mat');


% Expected system params
% Motor specs
R = 0.144; % ohms
km = 9.74; %oz in per amp
km = km * 0.0070615518333333; % Nm / amp
ke = 137; %rpm per volt
ke = ke * 2 * pi / 60; %rad per volt
% aproximate no
start_current = 3.5; % amps, about that
% Columb friction term
M_f = start_current * km;

% Inertia guess
J_sys = 0.18; % kg m^2, old system had 0.11 for brake alone

a22 = -km * ke / J_sys;
b21 = km / J_sys;
b22 = -M_f * R / J_sys;
c2 = -ke / R;
d21 = 1 / R;

parameters = {...
    'a22', a22; ...
    'b21', b21; ...
    'b22', b22; ...
    'c2', c2; 
    'd2', d21};
odefun = 'upper_pulley_dynamics_expanded';
fcn_type = 'c';
init_sys = idgrey(odefun, parameters, fcn_type);

opt = greyestOptions('InitialState', 'estimate', 'EnforceStability',... 
    true, 'Display', 'on');
est_sys = greyest(data, init_sys);

opt = compareOptions('InitialCondition', 'z');
compare(data, est_sys, Inf); %, opt);


expected_a = [0, 1; 0, a22]; 
expected_b = [0, 0; b21, b22];
expected_c = [1, 0;  c2, 0];
expected_d =  [0, 0; d21, 0];

%save('logs/one_dir_est_sys.mat', 'est_sys');
save('logs/sigmoid_sys_est_2.mat', 'est_sys');

