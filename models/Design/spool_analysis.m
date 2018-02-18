clear;

% 1/8 in pads , fully extended 
J_brake = 110933488.51; %grams * mm^2
% 1/4 in pads J_brake = 173139896.72; %grams * mm^2
J_brake = J_brake * 10^-3 * 10^-6;

%% Material properties used %% 
in_to_m = 0.0254;

material = 'Aluminium';

% 6061-T6 http://asm.matweb.com/search/SpecificMaterial.asp?bassnum=ma6061t6
% Might be optomistic
% refine these numbers
rho_aluminium = 2.70 * 10^-3 * 10^6; % kg/m^3
yeild_aluminium = 276 * 10^6; % Pa
shear_aluminium = 207 * 10^6; % Pa

% Steel pipe requirements from here might apply
% http://www.spsfence.com/Files/14-JMC-1410_collateral_Fence_Submittal_Data_Sheet_Updates_F1083_Sched_40_v2.pdf
yeild_steel = 30000; % psi
yeild_steel = yeild_steel * 6894.76; % Pa
shear_steel= 0.58 * yeild_steel; % wikipedia ratio
rho_steel = 8050; % kg/m3 upper_bound

% Motor demands
feed_length_m = 1.219; % m = 4 ft
feed_time_s = 1; % 1 s

if strcmp(material, 'Steel')
    yeild_material = yeild_steel;
    shear_material = shear_steel;
elseif strcmp(material, 'Aluminium')
    yeild_material = yeild_aluminium;
    shear_material = shear_aluminium;
else
   disp('Invalid material'); 
end

%% Dimensions considered. 
% 6 inch schedule 40 4.5in OD aluminium pipe
% radii =      6.625 / 2 * in_to_m; %0.02:0.002:0.10;
% thickness =   .280 * in_to_m; %0.002:0.0001:0.01;
% lengths =    6 * in_to_m; %0.1:0.01:0.1;

radii =       [4.500, 4.500,  4.500, 4.500, 5.0,  5.0 , 6.625];
thickness =  [.237,   0.337, .237,  0.337, .247, .247, .280];
lengths =      [6, 6,         8, 8,           6,    8,   5]; 

radii = radii / 2 * in_to_m;
thickness = thickness * in_to_m;
lengths = lengths * in_to_m;

% Stress requirements
bending_stresses = zeros(length(radii), 1);%, length(thickness), length(lengths));
hoop_stresses = zeros(length(radii), 1);%, length(thickness), length(lengths));
torsional_shears = zeros(length(radii), 1);%, length(thickness), length(lengths));

% Motor Stresses
max_torques = zeros(length(radii), 1);%, length(thickness), length(lengths));
max_powers = zeros(length(radii), 1);%, length(thickness), length(lengths));
max_omegas = zeros(length(radii), 1);%, length(thickness), length(lengths));
J_spools = zeros(length(radii), 1);
num_stacks = zeros(length(radii), 1);

for i = 1:length(radii)
    r2 = radii(i);
    r1 = r2 - thickness(i);
    l = lengths(i);
    [bending_stress, hoop_stress, torsional_shear] = expected_stresses(r2, r1, l, material);
    bending_stresses(i) = bending_stress;
    hoop_stresses(i) = hoop_stress;
    torsional_shears(i) = torsional_shear;
    
    [tau_max, P_max, omega_max] = motor_requirements(feed_length_m, feed_time_s, r2, r1, l, material, J_brake);
    max_torques(i) = tau_max;
    max_powers(i) = P_max;
    max_omegas(i) = omega_max;
    
    [J_spool, m_spool, max_r, num_stack] = spool_inertia_calc(r2, r1, l, material);
    J_spools(i) = J_spool;
    num_stacks(i) = num_stack;
    
end

bending_sf = yeild_aluminium ./ bending_stresses;
hoop_sf = yeild_aluminium ./ hoop_stresses;
torsional_sf = shear_aluminium ./ torsional_shears;


all_data = zeros(length(radii), 10);
all_data(:, 1) = radii;
all_data(:, 2) = thickness;
all_data(:, 3) = lengths;
all_data(:, 4) = J_spools;
all_data(:, 5) = num_stacks;
all_data(:, 6) = max_torques;
all_data(:, 7) = max_omegas;
all_data(:, 8) = max_powers;
all_data(:, 9) = hoop_sf;
all_data(:, 10) = bending_sf;


% Find the best option now. 

%min_power_option = 100000000;
% for i = 1:length(radii)
%     for j = 1:length(thickness)
%         for k = 1:length(lengths)
%             if (bending_sf(i, j, k) > 1.25 && ...
%                     hoop_sf(i, j, k) > 1.25 && ...
%                     torsional_sf(i, j, k) > 1.25 && ...
%                     max_powers(i, j, k) < min_power_option)
%                min_power_option = max_powers(i, j, k); 
%                i_best = i;
%                j_best = j;
%                k_best = k;
%             end
%         end 
%     end
% end
% 
% 
% 
% r_best = radii(i_best)
% t_best = thickness(j_best)
% l_best = lengths(k_best)
% 
% bending_sf_best = bending_sf(i_best, j_best, k_best)
% hoop_sf_best = hoop_sf(i_best, j_best, k_best)
% torsional_sf_best = torsional_sf(i_best, j_best, k_best)
% 
% best_power = max_powers(i_best, j_best, k_best)
% best_omega = max_omegas(i_best, j_best, k_best)
% best_torque = max_torques(i_best, j_best, k_best)
% 
% subplot(2, 1, 1);
% surf(thickness, radii, max_torques(:, :, 1));
% ylabel('Radii [m]');
% xlabel('Thickness [m]');
% zlabel('Max Torque Required [Nm]');
% subplot(2, 1, 2);
% surf(thickness, radii, max_powers(:, :, 1));
% ylabel('Radii [m]');
% xlabel('Thickness [m]');
% zlabel('Max Power Required [W]');
% figure;
% 
% subplot(2, 1, 1);
% surf(thickness, radii, hoop_sf(:, :, end));
% ylabel('Radii [m]');
% xlabel('Thickness [m]');
% zlabel('Hoop Stress S.F.');
% subplot(2, 1, 2);
% surf(thickness, radii, bending_sf(:, :, end));
% ylabel('Radii [m]');
% xlabel('Thickness [m]');
% zlabel('Bending Stress S.F.');


