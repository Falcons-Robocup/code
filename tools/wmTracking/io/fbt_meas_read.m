function measurements = fbt_meas_read(filename, varargin)
% read measurements from a file


% options
options.verbose    = false;
[options, args]    = getopts(options, varargin{:});

% load file
if options.verbose
    disp(sprintf('# loading %s ...', filename));
end
filedata           = fbt_meas_parse(filename);
measurements       = fieldata.measurements;

% convert to measurements struct
measurements       = [];
measurements.n     = size(measvalues, 1);
measurements.data  = measvalues(:, [1:11]);
if options.verbose
    disp(sprintf('# got %d measurements', measurements.n));
    tstart = measurements.data(1, 3);
    tend = measurements.data(end, 3);
    disp(sprintf('#   t_start = %s', fbt_time_float2str(tstart)));
    disp(sprintf('#   t_end   = %s', fbt_time_float2str(tend)));
    disp(sprintf('#   elapsed = %.2fs', tend - tstart));
end

% filter?
if numel(args)
    if options.verbose
        disp(sprintf('# filtering data ...'));
        disp(args); 
    end
    measurements_orig = measurements;
    [measurements, removed] = fbt_meas_filter(measurements, args{:});
    if options.verbose
        disp(sprintf('# %d measurements remaining, %d measurements filtered (%.1f%%)', measurements.n, removed.n, 100.0 * removed.n / measurements_orig.n));
    end
end

% add meta data
if options.verbose
    disp(sprintf('# extending data with cached matrices ...'));
end
measurements       = fbt_meas_addcart(measurements);

% sort on time
measurements       = fbt_meas_sort(measurements);

% cache 4x4 design matrix per measurement?
measurements       = fbt_meas_add_dmcache(measurements);
if options.verbose
    disp(sprintf('# done processing %s', filename));
end


