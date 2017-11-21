clear;

% Rope specs
T = 30; % Total rope length [m]
m = 0.063; % Mass [kg], from Sterling Slim Gym Climbing Rope - 10.1mm, 63 g
rho = m / T; % [kg / m], linear density of rope
D = 0.0101; % Rope diameter [m], 1.01 cm

% Spool specs
r = 0.15; % Spool radius [m]
l = 0.30; % Spool length [m]

% How many times can the rope be wrapped around the length without stacking
% on top of itself
windings_per_stack = floor(l / D);



% Rope rotational inertia [kg m^2]
rope_inertia = 0; 

r_curr = 0.15;
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
    m_stack = rho * length_of_stack;
    rope_inertia = rope_inertia + r_curr^2 * m_stack;
end

pipe_inertia = 488.71; % lg * in^2 
pipe_inertia = pipe_inertia * 0.000292639653; % kg m^2
total_inertia = rope_inertia + pipe_inertia; 
% Lol, all the inertia is from the pipe. 





