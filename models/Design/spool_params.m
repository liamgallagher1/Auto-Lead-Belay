clear;

% 6 series I assume
aluminium_rho = 2700; % kg / m^3
aluminium_break = 310 * 10^6; % Pa

steel_rho = 8050;
steel_break = 450 * 10^6; % Pa

% Thickness
t = 0.05; % 1 cm seems fine?
% Rope diameter
D = 0.0101; 

r_2s = 0.10:0.02:0.20;
ls = 0.10:0.04:0.30; 
ts = 0.05:0.05:2.5;

inertias = zeros(length(ls), length(r_2s));
% Safety factors with respect to hoop stress, assumings 8kn load

% Assuming constant stress
hoop_factor_const = zeros(length(ls), length(r_2s));
% Assuming concentrated pressure in two rope lengths
hoop_factor_two = zeros(length(ls), length(r_2s));


for i = 1:length(r_2s)
    r_2 = r_2s(i);
    for j = 1:length(ls)
        l = ls(j);
        [Izz, sigma_max_p] = spool_inertia_strenth(t, r_2, l, aluminium_rho);
        inertias(j, i) = Izz;
        
        % How do we distribute 8 kN? 
        % Overly optimistic
        pressure_const = -8000 / (2 * pi * r_2 * l);
        % Seems overly conservative 
        pressure_two = -8000 / (2 * pi * r_2 * 2 * D);
        
        hf_const = aluminium_break / (sigma_max_p * pressure_const);
        hf_two = aluminium_break / (sigma_max_p * pressure_two);
        hoop_factor_const(j, i) = hf_const; 
        hoop_factor_two(j, i) = hf_two; 
    end
end


subplot(3, 1, 1);
surf(r_2s, ls, inertias);
xlabel('radius [m]');
ylabel('length [m]');
title('Inertia');

subplot(3, 1, 2);
surf(r_2s, ls, hoop_factor_const);
xlabel('radius [m]');
ylabel('length [m]');
title('HF Const');

subplot(3, 1, 3);
surf(r_2s, ls, hoop_factor_two);
xlabel('radius [m]');
ylabel('length [m]');
title('HF Const');

