function measurements = fbt_meas_merge(varargin)
% merge measurements, sort in ascending time


% initialize output
measurements = varargin{1};

% only 2nd output?
if (nargin == 2) && isempty(measurements)
    measurements = varargin{2};
    return
end

% make sure all fields are present
measurements = fbt_meas_addcart(measurements);

% merge
for iarg = 2:nargin
    m_other = fbt_meas_addcart(varargin{iarg});
    measurements.data = [measurements.data; m_other.data];
end
    
% sort
measurements = fbt_meas_sort(measurements);

