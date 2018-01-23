function encoder_counts = tics(time, velocity_profile, PPR)
% Outputs the encoder counts to be measured by the encoder at each time step
% TODO add noise, radius parameters and such
% time [T x 1] vector of times
% velocity_profile [T x 1] vector of speeds, [rad / s] for now
% PPR number of pulses per revolution of enocer. Assume quadrature encoder

T = length(time);

encoder_counts = zeros(T, 1);

% vector of radial positions
positions = zeros(T, 1);

for t = 2:T
    % Maybe this is stupid
    % Increment position using second order taylor expansion
    positions(t) = positions(t - 1) + (time(t) - time(t - 1)) * velocity_profile((t - 1)) ...
        + 0.5 * (time(t) - time(t - 1))^2 * (velocity_profile(t) - velocity_profile(t - 1));
    encoder_counts(t) = floor(positions(t) * (PPR * 4) / 2 / pi);
end

end

