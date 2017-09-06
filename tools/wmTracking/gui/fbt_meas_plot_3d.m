function fbt_meas_plot_3d(measurements, varargin)
% plot measurements in 3d scene


% settings
options.colors        = fbt_robot_colors;
options.axes          = [];
options.ballstyle     = '*';
options.robotstyle    = '+';

% parse options
[options, args]       = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% extract vectors
id                    = measurements.data(:, fbt_col_idx('id', measurements));
bx                    = measurements.data(:, fbt_col_idx('bx', measurements));
by                    = measurements.data(:, fbt_col_idx('by', measurements));
bz                    = measurements.data(:, fbt_col_idx('bz', measurements));
cx                    = measurements.data(:, fbt_col_idx('cx', measurements));
cy                    = measurements.data(:, fbt_col_idx('cy', measurements));
cz                    = measurements.data(:, fbt_col_idx('cz', measurements));
% TODO use cphi

% loop over robots for color grouping
hold(options.axes, 'on');
for irobot = 1:6
    imeas             = find(id == irobot);
    if numel(imeas)
        plot3(options.axes, bx(imeas), by(imeas), bz(imeas), [options.colors(irobot) options.ballstyle], 'MarkerSize', 8);
        plot3(options.axes, cx(imeas), cy(imeas), cz(imeas), [options.colors(irobot) options.robotstyle], 'MarkerSize', 8);
    end
end
