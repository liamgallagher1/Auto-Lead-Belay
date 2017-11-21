function [J_spool, m_spool, max_r] = spool_inertia_calc(r2, r1, l, material)
% Calculates the mass moment of inertia and mass of spool. 
% TODO add materials

if strcmp(material, 'Aluminium')
    rho = 2.700 * 10^-3 * (10^2)^3; %% Aluminim vs Steel. 
elseif strcmp(material, 'Steel')
    rho = 8050;
else
    disp('Invalid Material')
    return
end


% Rope specs
T = 30; % Total rope length [m]
rope_mass = 0.063; % Mass [kg], from Sterling Slim Gym Climbing Rope - 10.1mm, 63 g
rho_rope = rope_mass / T; % [kg / m], linear density of rope
D = 0.0101; % Rope diameter [m], 1.01 cm

% How many times can the rope be wrapped around the length without stacking
% on top of itself
windings_per_stack = floor(l / D);

% Rope rotational inertia [kg m^2]
rope_inertia = 0; 
r_curr = r2 + D / 2;
length_remaining = T; 
num_stacks = 0;
while (length_remaining > 0)
    % Wrap the rope around the spool again
    num_stacks = num_stacks + 1;
    length_of_stack = r_curr * 2 * pi * windings_per_stack;
    % But only as far as it will go 
    new_length_remaining = max(length_remaining - length_of_stack, 0);
    length_of_stack = length_remaining - new_length_remaining; 
    length_remaining = new_length_remaining;
    r_curr = r_curr + D; 
    
    % I = sum r^2 dm
    m_stack = rho_rope * length_of_stack;
    rope_inertia = rope_inertia + r_curr^2 * m_stack;
end

max_r = r_curr - D/2;

J_pipe = pi * rho * l / 2 * (r2^4 - r1^4);  
m_pipe = pi * rho * l * (r2^2 - r1^2); 

J_spool = rope_inertia + J_pipe;
m_spool = m_pipe + rope_mass;

end

