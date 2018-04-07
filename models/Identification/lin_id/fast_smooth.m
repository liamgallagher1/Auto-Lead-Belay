function [coefs, start_indx, end_indx, breaks] = ...
    fast_smooth(x, y, smooth_weight, num_segments)
%FAST_SMOOTH Summary of this function goes here
%   Detailed explanation goes here

seg_length = floor(length(x) / num_segments);
seg_indx = zeros(num_segments, 2);
seg_indx(1, 1) = 1;
seg_indx(1, 2) = seg_length + 1;


for i = 2:num_segments
   seg_indx(i, 1) = seg_indx(i - 1, 1) + seg_length;
   seg_indx(i, 2) = seg_indx(i - 1, 2) + seg_length;
end
seg_indx(end, 2) = length(x);

coefs = zeros(length(x), 4);
breaks = zeros(length(x), 1);

for i = 2:num_segments-1
   sub_x = x(seg_indx(i-1, 1):seg_indx(i+1, 2));
   sub_y = y(seg_indx(i-1, 1):seg_indx(i+1, 2));
   seg_fit = fit(sub_x, sub_y,'smoothingspline', 'SmoothingParam', smooth_weight);
   rel_start = seg_indx(i, 1) - seg_indx(i - 1, 1) + 1;
   rel_stop = seg_indx(i, 2)-1  - seg_indx(i -1, 1) + 1;
   coefs(seg_indx(i, 1):seg_indx(i, 2)-1, :) = seg_fit.p.coefs(rel_start:rel_stop, :);
   breaks(seg_indx(i, 1):seg_indx(i, 2), :) = seg_fit.p.breaks(:, rel_start:rel_stop+1)';
end

start_indx = seg_indx(2, 1);
end_indx = seg_indx(num_segments-1, 2)-1;
coefs = coefs(start_indx:end_indx, :);
breaks = breaks(start_indx:end_indx+1, :);




end

