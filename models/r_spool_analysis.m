clear;
clf
close all

material = 'Aluminium';

in_to_m = 0.0254; 

R_spool_in = 4:0.05:12;
R_spool = in_to_m * R_spool_in;
h = in_to_m * 8;
t = in_to_m * 0.2;
% Like section 40

v_rope = 1.25; % A little faster than 4 ft per second
a_rope = 1.25/0.5; % achieved after a second

I_brake = 0.0738245; %kg m^2

torques = zeros(length(R_spool), 1);
powers = zeros(length(R_spool), 1);


for i = 1:length(R_spool)
    r2 = R_spool(i);
    r1 = r2 - t;
    [I_spool, m_spool, max_r] = spool_inertia_calc(r2, r1, h, material);
    T_m = a_rope / r2 * (I_spool + I_brake);
    P_m = T_m * v_rope / r2;
    torques(i) = T_m;
    powers(i) = P_m;
end

%subplot(3, 1, 1);
rope_movement = [0, 1.25, 0];
time = [0, 0.5, 1];
plot(time, rope_movement, 'LineWidth', 3.5);
ylabel('Commanded rope speed [m/s]');
xlabel('Time [s]');

figure;
subplot(2, 1, 1);
plot(R_spool_in, torques, 'b', 'LineWidth', 3.5);
xlabel('Spool Radius [in]');
ylabel('Maximum Motor torque [Nm]');


subplot(2, 1, 2);
plot(R_spool_in, powers, 'r', 'LineWidth', 3.5);
xlabel('Spool Outer Radius [in]');
ylabel('Maximum Motor power [W]');






