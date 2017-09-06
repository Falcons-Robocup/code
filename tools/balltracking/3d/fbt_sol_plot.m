function fbt_sol_plot(solution, varargin)
% plot a solution


% assume plot is already open
hold on;

% options
options.scaling = 0.3;
options.color   = 'k';
options.axes    = gca;

% parse options
[options, args] = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% ball location
plot3(options.axes, solution.x, solution.y, solution.z, [options.color 'o'], 'MarkerSize', 12);
% ball speed vector
s = options.scaling;
quiver3(options.axes, solution.x, solution.y, solution.z, s*solution.vx, s*solution.vy, s*solution.vz, [options.color '-']);
