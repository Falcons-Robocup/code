function [fighandle, leg] = fbt_meas_plot(measurements, varargin)
% plot measurements 


% settings
options.colors        = fbt_robot_colors;
options.axes          = [];
options.reusefig      = false;
options.whatstyle     = {'x', '+', '*', 'o', 's', 'd', 'v', '^', 'x', '+', '*', 'o', 's'};
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
t                     = measurements.data(:, fbt_col_idx('t', measurements));
if isfield(measurements, 't0')
    t                 = t - measurements.t0;
end
id                    = measurements.data(:, fbt_col_idx('id', measurements));
data_to_plot          = setdiff(measurements.columns, {'t', 'id'});
numcolumns            = size(measurements.data, 2);

% loop over robots for color grouping, for speed (looping over each measurement is way too slow)
% it is a design choice in here to plot robot-specific data with different colors
% (alternatively we could have put the responsibility of data selection at client)
for irobot = 1:6
    imeas             = find(id == irobot);
    if numel(imeas)
        for ivalue = 1:numel(data_to_plot)
            lbl       = data_to_plot{ivalue};
            idx       = fbt_col_idx(lbl, measurements);
            if idx <= numcolumns
                values    = measurements.data(:, idx);
                values    = values(imeas);
                plot(options.axes, t(imeas), values, [options.colors(irobot) options.whatstyle{ivalue}]);
                sub_textlabel(options, irobot, lbl, t(end), values(end));
            else
                warning(['data column not present for ' lbl]);
            end
        end
    end
end

        
function sub_textlabel(options, irobot, what, tpos, ypos)
    if options.textlabels
        tt = text(tpos, ypos + options.text_y_offset, sprintf('%sR%d', what, irobot));
        set(tt, 'Parent', options.axes);
        set(tt, 'Color', options.colors(irobot));
    end
