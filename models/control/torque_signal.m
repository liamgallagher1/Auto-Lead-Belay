function [T_m] = torque_signal(t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if (t < 0.5)
    T_m = 0.5;
else
    T_m = -0.5;
end

end

