function [M] = min_jerk_matrix(n, t0, t_curr)
% Calculates a matrix that gives the value of the the square of jerk
% integrated between t0 and t_curr, 
% as a quadratic function of the polynomial parameters P = [p0, p1 .. pn]'. 
% x(t) = p0 + p1 * t ...
% sum jerk^2 = P' * M * P
% JK P is [p_n, p_n-1 ... p_0]' 

M = zeros(n + 1, n + 1);

for l = 3:n
    for p = 3:n
        derv = 1 / (l + p - 6 + 1);
        const = derv * l*(l-1)*(l-2)*p*(p-1)*(p-2);
        t_curr_p = t_curr^(l + p - 5);
        t_0_p = t0^(l  + p - 5);
        val = const * (t_curr_p - t_0_p);
        if (l == p) % diagnal elements
            M(l+1, l+1) = val;
        else
            % Keep it symmetric
            M(l+1, p+1) = M(l+1, p+1) + val/2;
            M(p+1, l+1) = M(p+1, l+1) + val/2;
        end
    end
end

% Jk, I want to get this flipped along the diagnal. 
M = rot90(M, 2);

end

