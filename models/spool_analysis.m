clear;

%% Material properties used %% 

material = 'Steel';

% 6061-T6 http://asm.matweb.com/search/SpecificMaterial.asp?bassnum=ma6061t6
% Might be optomistic
rho_aluminium = 2.70 * 10^-3 * 10^6; % kg/m^3
yeild_aluminium = 310 * 10^6; % Pa
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
    shear_material = shear_alumininum;
else
   disp('Invalid material'); 
end

%% Dimensions considered. 
radii = 0.02:0.002:0.10;
thickness = 0.002:0.0001:0.01;
lengths = 0.1:0.01:0.1;

% Stress requirements
bending_stresses = zeros(length(radii), length(thickness), length(lengths));
hoop_stresses = zeros(length(radii), length(thickness), length(lengths));
torsional_shears = zeros(length(radii), length(thickness), length(lengths));

% Motor Stresses
max_torques = zeros(length(radii), length(thickness), length(lengths));
max_powers = zeros(length(radii), length(thickness), length(lengths));
max_omegas = zeros(length(radii), length(thickness), length(lengths));


for i = 1:length(radii)
    r2 = radii(i);
    for j = 1:length(thickness)
        r1 = r2 - thickness(j);
        for k = 1:length(lengths)
            l = lengths(k);
            [bending_stress, hoop_stress, torsional_shear] = expected_stresses(r2, r1, l, material);
            bending_stresses(i, j, k) = bending_stress;
            hoop_stresses(i, j, k) = hoop_stress;
            torsional_shears(i, j, k) = torsional_shear;
            
            [tau_max, P_max, omega_max] = motor_requirements(feed_length_m, feed_time_s, r2, r1, l, material);
            max_torques(i, j, k) = tau_max;
            max_powers(i, j, k) = P_max;
            max_omegas(i, j, k) = omega_max;
            
            [J_spool, m_spool, max_r] = spool_inertia_calc(r2, r1, l, material)
            
        end 
    end
end

bending_sf = yeild_aluminium ./ bending_stresses;
hoop_sf = yeild_aluminium ./ hoop_stresses;
torsional_sf = shear_aluminium ./ torsional_shears;


% Find the best option now. 

min_power_option = 100000000;

for i = 1:length(radii)
    for j = 1:length(thickness)
        for k = 1:length(lengths)
            if (bending_sf(i, j, k) > 1.25 && ...
                    hoop_sf(i, j, k) > 1.25 && ...
                    torsional_sf(i, j, k) > 1.25 && ...
                    max_powers(i, j, k) < min_power_option)
               min_power_option = max_powers(i, j, k); 
               i_best = i;
               j_best = j;
               k_best = k;
            end
        end 
    end
end

r_best = radii(i_best)
t_best = thickness(j_best)
l_best = lengths(k_best)

bending_sf_best = bending_sf(i_best, j_best, k_best)
hoop_sf_best = hoop_sf(i_best, j_best, k_best)
torsional_sf_best = torsional_sf(i_best, j_best, k_best)

best_power = max_powers(i_best, j_best, k_best)
best_omega = max_omegas(i_best, j_best, k_best)
best_torque = max_torques(i_best, j_best, k_best)

subplot(2, 1, 1);
surf(thickness, radii, max_torques(:, :, 1));
ylabel('Radii [m]');
xlabel('Thickness [m]');
zlabel('Max Torque Required [Nm]');
subplot(2, 1, 2);
surf(thickness, radii, max_powers(:, :, 1));
ylabel('Radii [m]');
xlabel('Thickness [m]');
zlabel('Max Power Required [W]');
figure;

subplot(2, 1, 1);
surf(thickness, radii, hoop_sf(:, :, end));
ylabel('Radii [m]');
xlabel('Thickness [m]');
zlabel('Hoop Stress S.F.');
subplot(2, 1, 2);
surf(thickness, radii, bending_sf(:, :, end));
ylabel('Radii [m]');
xlabel('Thickness [m]');
zlabel('Bending Stress S.F.');


