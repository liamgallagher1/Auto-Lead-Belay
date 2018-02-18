function [t1, t2] = min_thickness(r2, material, SF)
%MIN_THICKNESS Returns the minimum thickness of a spool for a given safety
% factor material, force. Only considers hoop stress
d_r = 0.01; % rope thickness
n_r = 1; % number of wraps rope is distributed around
F_max = 8500; % N
F_max = F_max * SF; 
P_max = F_max / pi / 2 / r2 / n_r / d_r;

yeild_aluminium = 276 * 10^6; % Pa
% Steel pipe requirements from here might apply
% http://www.spsfence.com/Files/14-JMC-1410_collateral_Fence_Submittal_Data_Sheet_Updates_F1083_Sched_40_v2.pdf
yeild_steel = 30000; % psi
yeild_steel = yeild_steel * 6894.76; % Pa

if strcmp(material, 'Steel')
    yeild_material = yeild_steel;
elseif strcmp(material, 'Aluminium')
    yeild_material = yeild_aluminium;
else
   disp('Invalid material'); 
end

sigma_max = yeild_material; 


alpha = sigma_max / P_max; 
r1 = r2 * sqrt( (alpha - 1) / (alpha + 1)); 
t1 = r2 - r1;

% 
% a = 1 + alpha;
% b = -2 * r2 * (1 + alpha); 
% c = 2 * r2^2; 
% 
% t1 = (-b - sqrt(b^2 - 4 * a * c)) / 2 / a;
% t2 = (-b + sqrt(b^2 - 4 * a * c)) / 2 / a; 

end

