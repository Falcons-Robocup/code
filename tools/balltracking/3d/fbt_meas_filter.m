function [measurements, other] = fbt_meas_filter(measurements, varargin)
% filter measurements



% default filter criteria
% (factored out for reuse in callers, to avoid costly option parsing)
options       = fbt_filter_opts;

% this is a performance hotspot -- do not parse options if already a struct is provided
if (nargin > 1) && isstruct(varargin{1})
    options   = varargin{1};
else
    % parse options
    [options, args] = getopts(options, varargin{:});
    assert(numel(args) == 0); % don't know what to do with unparsed arguments
end
% TODO subsampling?

% unpack measurements
id    = measurements.data(:, 1);
cam   = measurements.data(:, 2);
t     = measurements.data(:, 3);
xc    = measurements.data(:, 4);
yc    = measurements.data(:, 5);
zc    = measurements.data(:, 6);
phi   = measurements.data(:, 7);
az    = measurements.data(:, 8);
el    = measurements.data(:, 9);
r     = measurements.data(:, 10);
conf  = measurements.data(:, 11);
metadatapresent = (size(measurements.data, 2) > 11);
if metadatapresent
    bx    = measurements.data(:, 12);
    by    = measurements.data(:, 13);
    bz    = measurements.data(:, 14);
end

% initialize 
idx   = 1:measurements.n;

% filter on robot
if options.robot
    idx2 = find(id == options.robot);
    idx  = intersect(idx, idx2);
end

% filter on time
idx2  = find((t >= options.tmin) & (t <= options.tmax));
idx   = intersect(idx, idx2);

% filter on absolute position
if metadatapresent
    %[bx, by, bz] = fct_ball2fcs(xc, yc, zc, phi, az, el, r); % NOTE: from now on we assume data is already there
    idx2  = find((bx >= options.xmin) & (bx <= options.xmax) & (by >= options.ymin) & (by <= options.ymax) & (bz >= options.zmin) & (bz <= options.zmax));
    idx   = intersect(idx, idx2);
end

% apply filter
[measurements, other] = fbt_meas_reindex(measurements, idx);
