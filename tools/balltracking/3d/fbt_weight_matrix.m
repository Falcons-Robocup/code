function W = fbt_weight_matrix(settings, measurements)
% weight matrix, to reduce influence of noisy distance measurements

n      = measurements.n;
idx    = 4 * [1:n] - 2;
d      = ones(4*n, 1);
d(idx) = settings.solver.core.weight;
assert(n < 500); % avoid memory/performance problem
W      = diag(d);
