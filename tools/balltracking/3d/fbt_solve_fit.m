function [solution, residuals] = fbt_solve_fit(settings, measurements, tcurr)
% solve by direct fit


% determine measurement spread, if too small, then fallback to position-only fit
t                = measurements.data(:, fbt_meas_idx('t'));
tmin             = min(t);
tmax             = max(t);
timespread       = tmax - tmin;
if nargin < 3
    tcurr        = tmax;
end
if (tmax > tcurr)
    warning(sprintf('requested a fit with measurements from the future?!'));
end
if (timespread > 2.1)
    error(sprintf('requested a fit with high measurement time spread (%.1fs)', timespread));
end
if settings.solver.core.speed && (timespread < settings.solver.core.minvdt)
    % disable for the remainder of this function
    settings.solver.core.speed = 0;
end

% construct linear system of equations A * p = b with p as parameters to be fitted
A                = fbt_design_matrix(settings, measurements, tcurr);
b                = fbt_meas_vector  (settings, measurements);
W                = fbt_weight_matrix(settings, measurements);
% constraints are not (yet?) used
%Q                = eye(size(A, 2)); % TODO? C = fbt_constraint_matrix; Q = null(C);

% solve
p                = (W * A) \ (W * b);
% reduced form (without weighting):
%   p = A \ b    
% full form (including 'constraints'):
%   p = Q * ((W * A * Q) \ (W * b));

% calculate residuals
residuals        = b - A * p;
solution.quality = fbt_solve_fit_quality(residuals);

% set output
solution.x       = p(1);
solution.y       = p(2);
solution.z       = p(3);
if settings.solver.core.speed
    solution.vx  = p(5);
    solution.vy  = p(6);
    solution.vz  = p(7);
else
    solution.vx  = 0;
    solution.vy  = 0;
    solution.vz  = 0;
end

