clear;

T = 40; % Total rope length [m]
l = 0.30; % Spool length [m]
r = 0.15; % Spool radius [m]

b =  1 + 2 * r;
c = - T / pi / l;

n = (-b + sqrt(b^2 - 4 * c)) / 2;

r_outer = r + n * 0.01;
