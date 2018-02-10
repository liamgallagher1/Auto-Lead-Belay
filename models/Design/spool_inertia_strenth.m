function [Izz, simga_max_p] = spool_inertia_strenth(...
    t, ...   % [m] pipe thickness
    r_2, ... % [m] outer radius
    l, ...   % [m] Length of the pipe
    rho)     % [kg/m^3] density of the metal
% Outputs an estimate of the mass moment of inertia of a spool, 
% pipe and rope. 
% Outputs an estimate of the hoop stress / external pressure ratio

% Rope specs
T = 30; % Total rope length [m]
m = 0.063; % Mass [kg], from Sterling Slim Gym Climbing Rope - 10.1mm, 63 g
rho_rope = m / T; % [kg / m], linear density of rope

D = 0.0101; % Rope diameter [m], 1.01 cm


% How many times can the rope be wrapped around the length without stacking
% on top of itself
windings_per_stack = floor(l / D);

% Rope rotational inertia [kg m^2]
rope_inertia = 0; 

r_curr = r_2;
length_remaining = T; 
num_stacks = 0;

while (length_remaining > 0)
    % Wrap the rope around the spool again
    num_stacks = num_stacks + 1;
    length_of_stack = (r_curr + D / 2) * 2 * pi * windings_per_stack;
    % But only as far as it will go 
    new_length_remaining = max(length_remaining - length_of_stack, 0);
    length_of_stack = length_of_stack - new_length_remaining; 
    length_remaining = new_length_remaining;
    r_curr = r_curr + D; 
    
    % I = sum r^2 dm
    m_stack = rho_rope * length_of_stack;
    rope_inertia = rope_inertia + r_curr^2 * m_stack;
end


pipe_mass = rho * l * pi * (r_2^2 - (r_2 - t)^2); 
pipe_inertia = pipe_mass * r_2^2 * ( 1 - t + t^2 / 2); 
Izz = rope_inertia + pipe_inertia; 

simga_max_p = -2 * r_2^2 / (r_2^2 - (r_2 - t)^2);

end

