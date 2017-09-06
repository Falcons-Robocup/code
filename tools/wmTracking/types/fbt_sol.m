function solutions = fbt_sol(n)
% create a solutions struct


if nargin == 0
    n = 0;
end

% columns
fn = {'t', 'id', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'conf'};
solutions.columns = fn;
solutions.data = zeros(n, numel(fn));
solutions.n = n;

