function fbt_sol_plot_time(solution, varargin)
% plot solution series against time


% assume plot is already open
hold on;

% options
options.colors  = {'r', 'b', 'g'};
options.axes    = gca;

% parse options
[options, args] = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% ball location against time
t = solution.t - min(solution.t);
plot(options.axes, t, solution.x, [options.colors{1} 'o'], 'MarkerSize', 12);
plot(options.axes, t, solution.y, [options.colors{2} 'o'], 'MarkerSize', 12);
plot(options.axes, t, solution.z, [options.colors{3} 'o'], 'MarkerSize', 12);

% TODO ball speed 
