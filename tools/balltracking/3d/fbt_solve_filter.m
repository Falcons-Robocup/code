function varargout = fbt_solve_filter(varargin)
% solution smoothener
% for now only used in KPI's, to evaluate stability
% could also be used in solver, to improve stability


% which filter types are supported?
filter_types       = {'none', 'minmax', 'mamsd'};
default_filter     = 'mamsd';
if (nargin == 1) && strcmp(varargin{1}, 'list')
    varargout{1}   = filter_types;
    return
end

% options parsing
options.filtertype = default_filter;
options.filterlen  = 5;
options.trackerid  = 1; % TODO if 0, then do for each tracker?
[options, args]    = getopts(options, varargin{:});
assert(ismember(options.filtertype, filter_types));
solutions          = args{1}; % required argument

% do the filtering
if ~isfield(solutions, 'filtered')
    solutions.filtered = [];
end
if strcmp(options.filtertype, 'none')
    % nothing to do
    1;
elseif strcmp(options.filtertype, 'minmax')
    solutions      = sub_foreach(solutions, options, 'min', @sub_min);
    solutions      = sub_foreach(solutions, options, 'max', @sub_max);
elseif strcmp(options.filtertype, 'mamsd')
    % TODO: inefficient re-calculation of MA within sub_msd
    solutions      = sub_foreach(solutions, options, 'ma',  @sub_ma);
    solutions      = sub_foreach(solutions, options, 'msd', @sub_msd);
else
    error(sprintf('filter ''%s'' not implemented', options.filtertype));
end

% set output
varargout{1}       = solutions;



% end of main, sub functions below
function so = sub_foreach(si, options, name, callback)
    so = si;
    % extend solutions struct si with filtered data
    fn             = {'x', 'y', 'z', 'vx', 'vy', 'vz'};
    % TODO: analyze each tracker if 0 is given
    for itracker = options.trackerid
        % calculate for each coordinate
        idx        = find(si.id == itracker);
        so.filtered.t = si.t(idx); % TODO recalculating; also not robust for multiple trackers
        for ifield = 1:numel(fn)
            fv     = [];
            if isfield(so.filtered, name)
                fv = getfield(so.filtered, name);
            end
            so.filtered.(name) = setfield(fv, fn{ifield}, feval(callback, options, getfield(si, fn{ifield})));
        end
    end

function vo = sub_min(options, vi)
    vo = min(vi);
    
function vo = sub_max(options, vi)
    vo = max(vi);
    
function vo = sub_ma(options, vi)
    % borrowed from ASMLtoolbox mamsdfilter
    window         = repmat(1, [1 options.filterlen]) ./ options.filterlen;
    ma             = filter(window, 1, vi);        % filter input
    vo             = ma;
    
function vo = sub_msd(options, vi)
    % borrowed from ASMLtoolbox mamsdfilter
    window         = repmat(1, [1 options.filterlen]) ./ options.filterlen;
    ma             = filter(window, 1, vi);        % filter input
    ma2            = filter(window, 1, vi.^2);     % filter squared input
    msd            = sqrt(max(ma2 - ma.^2, 0));    % calculate standard deviation and be tolerant for numerical roundings causing negative result
    vo             = msd;
