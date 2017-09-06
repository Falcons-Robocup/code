function fbt_sol_plot_time(solutions, varargin)
% plot a solutions struct against time



% settings
options.color         = 'k';
options.axes          = [];
options.reusefig      = false;
options.whatstyle     = {'x', '+', '*', 'o', 's', 'd', 'v'};
options.textlabels    = false;
options.text_y_offset = 0.1;

% parse options
[options, args]       = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% prepare to plot
if ~options.reusefig
    fighandle         = figure;
    options.axes      = gca;
else
    fighandle         = gcf;
end
assert(~isempty(options.axes));
hold(options.axes, 'on');
t                     = solutions.data(:, fbt_col_idx('t', solutions));
if isfield(solutions, 't0')
    t                 = t - solutions.t0;
end
data_to_plot          = setdiff(solutions.columns, {'t'});
numcolumns            = size(solutions.data, 2);

% plot
for ivalue = 1:numel(data_to_plot)
    lbl               = data_to_plot{ivalue};
    idx               = fbt_col_idx(lbl, solutions);
    if idx <= numcolumns
        values        = solutions.data(:, idx);
        plot(options.axes, t, values, [options.color options.whatstyle{ivalue}]);
    else
        warning(['data column not present for ' lbl]);
    end
end

