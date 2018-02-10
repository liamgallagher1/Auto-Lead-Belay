clear;


in_to_m = 0.0254;
lbin2_to_kgm2 = 0.0002926397;
lb_to_kg = 0.453592;

steel_density = 8050; % [Kg/m^3]

% Calculate rode inertia
L_axel = 16 * in_to_m;
r_axel = 1.25 / 2 * in_to_m;

J_axel = pi * steel_density * L_axel / 2 * r_axel^4;


% Brake inertia
J_brake_slide_in = 50.01 * lbin2_to_kgm2;
J_brake_slide_out = 52.91 * lbin2_to_kgm2; 
d_slide = 2.76 * in_to_m; 
delta_J_brake = J_brake_slide_out - J_brake_slide_in; 
% TODO this is wrong
% m_slide = delta_J_brake / d_slide^2 / 3; % parallel axis theorem?

% blade calculations, 1/8 in
m_blade = 0.43 * lb_to_kg;
J_blade_com = 5.51 * lbin2_to_kgm2; % inertia of blade itself
r_out = 10 * in_to_m; % outer radius of blade when fully expanded
r_com_out = (4.72 + 2.83) * in_to_m; % Radius of blade COM when fully expanded
r_com_in = r_com_out - d_slide; % All the way in



n_blades = 3; 
thickness_in = 0.125:0.025:0.4; 
m_blades = m_blade * thickness_in / 0.125; 
r_fixed = 0:0.005:d_slide; % radius along the slide
% Neglects the slider / connector mass but hopefully its fine
J_r = J_axel + J_brake_slide_in + n_blades * (J_blade_com +  m_blades' * (r_fixed + r_com_in).^2); 

% K divided by mass
v_sat = 4; % [m/s] spring saturation speed
r_spool = 8 / 2 * in_to_m;
omega_sat = v_sat / r_spool;
omega = 0:1:omega_sat;
K_m = omega_sat^2  /(1 + r_com_in/(d_slide + r_com_in)); % [N / M]
K_m = ((r_com_in + d_slide) * omega_sat^2) / (d_slide);
K = (m_blades * (r_com_in + d_slide) * omega_sat) / (d_slide); 

r_omega = K_m * r_com_in ./ max((K_m - omega.^2), K_m * r_com_in / (d_slide + r_com_in));
J_omega = J_axel + J_brake_slide_in + n_blades * (J_blade_com +  m_blades' * (r_omega).^2); 


clf;
subplot(1, 2, 1); 
surf(r_fixed / in_to_m, thickness_in, J_r);
xlabel('Blade Spring Extension [in]');
ylabel('Blade Thickness [in]');
title('Brake system inertia [kg m^2]');
subplot(1, 2, 2);
surf(omega * v_sat / omega_sat, thickness_in, J_omega);
xlabel('Rope Velocity [m/s]');
ylabel('Blade Thickness [in]');
title('Brake system inertia [kg m^2], spool diam = 6in, spring saturation at 4 m/s');






