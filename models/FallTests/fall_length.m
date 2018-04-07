function height = fall_length(inital_rope_out, inital_height, theta)

r0 = 0.185789405425864;
a = -5.492535985428214e-06;

inital_theta = -1/a * log(1 - a / r0 * inital_rope_out);
rope_out = r0 / a - r0/a * exp(-a * (inital_theta + theta));
fall_length = rope_out - inital_rope_out;
height = inital_height - fall_length;
end

