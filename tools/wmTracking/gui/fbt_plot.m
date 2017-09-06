function fbt_plot(data, varargin)
% main plotter to visualize any data, which may represent measurements or solutions
% standard one gets a 2D plot of data over time; there is also a 3D option ('-space')
% to plot only a subset of data, use fbt_filter - this way we can keep this plotter function clean

TODO remove

% settings
options.space         = false;

% parse options
[options, args]       = getopts(options, varargin{:});

% guess what kind of data we got
data_type             = 'solutions';
if ismember('id', data.columns)
    data_type         = 'measurements';
end

% call applicable plotter
if strcmp(data_type, 'measurements')
    if ~options.space
        fbt_meas_plot(data, args{:});
    else
        % TODO select bx,by,bz, then call fbt_3d_plot
        warning('going to space not supported');
        1;
    end
else
    if ~options.space
        fbt_sol_plot(data, args{:});
    else
        warning('plotting solutions in 3D not supported');
    end
end

