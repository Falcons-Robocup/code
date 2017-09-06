function A = fbt_design_matrix(settings, measurements, tcurr)
% create design matrix, used in fbt_solve


% assume matrix parts are already cached (in measurements, see fbt_design_matrix_cache)
t     = measurements.data(:, 3);
oidx  = 15:30;

% number of measurements
n     = measurements.n;

% construct age vector
tmin  = min(t);
tmax  = max(t);
age   = t - tcurr;

% construct matrix based on cache
A1    = measurements.data(:, oidx);
A2    = reshape(A1', [1 n*4*4]);
A     = reshape(A2', [4 4*n])';

% factor speed from solution into current position
if settings.solver.core.speed
    T = diag(reshape(repmat(age', [4 1]), 4*n, 1));
    A = [A T*A];
end
