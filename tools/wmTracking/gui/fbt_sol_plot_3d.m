function fbt_sol_plot_3d(solutions, varargin)
% plot a solutions struct in 3D


% options
options.scaling = 0.3;
options.color   = 'k';
options.axes    = gca;

% parse options
[options, args] = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% extract vectors
x               = solutions.data(:, fbt_col_idx('x', solutions));
y               = solutions.data(:, fbt_col_idx('y', solutions));
z               = solutions.data(:, fbt_col_idx('z', solutions));
vx              = solutions.data(:, fbt_col_idx('vx', solutions));
vy              = solutions.data(:, fbt_col_idx('vy', solutions));
vz              = solutions.data(:, fbt_col_idx('vz', solutions));

% ball location
hold(options.axes, 'on');
plot3(options.axes, x, y, z, [options.color 'o'], 'MarkerSize', 12);
% ball speed vector
s = options.scaling;
quiver3(options.axes, x, y, z, s*vx, s*vy, s*vz, [options.color '-']);
