clear;

% 1/8 in pads , fully extended 
%J_brake = 110933488.51; %grams * mm^2
% 1/4 in pads J_brake = 173139896.72; %grams * mm^2
%J_brake = J_brake * 10^-3 * 10^-6;
J_brake = 0.0471;

%% Material properties used %% 
in_to_m = 0.0254;

material = 'Aluminium';

% 6061-T6 http://asm.matweb.com/search/SpecificMaterial.asp?bassnum=ma6061t6
% Might be optomistic
% refine these numbers
rho_aluminium = 2.70 * 10^-3 * 10^6; % kg/m^3
yeild_aluminium = 276 * 10^6; % Pa
shear_aluminium = 207 * 10^6; % Pa

% Steel pipe requirements from here might apply, 305
% http://asm.matweb.com/search/SpecificMaterial.asp?bassnum=mq304a
%yeild_steel = 30000; % psi
yeild_steel = 215 * 10^6; % Pa
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

SF = 4;

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

% Minimize somewhat
min_thicknesses = zeros(length(radii), 1); 
min_J_spool = zeros(length(radii), 1); 
min_hoop_sf = zeros(length(radii), 1); 

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
    
    t = min_thickness(r2, material, SF);
    min_thicknesses(i) = t;
    [min_J, min_m_spool] = spool_inertia_calc(r2, r2 - t, l, material);
    min_J_spool(i) = min_J;
    [min_bending_stress, min_hoop_stress, min_torsional_shear] = expected_stresses(r2, r2 - t, l, material);
    min_hoop_sf(i) = yeild_material / min_bending_stress; 
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
all_data(:, 11) = min_thicknesses;
all_data(:, 12) = min_J_spool;
all_data(:, 13) = min_hoop_sf;


