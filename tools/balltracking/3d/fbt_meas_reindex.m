function [measurements, other] = fbt_meas_reindex(measurements, idx)
% take indexed subset of measurements


% calculate remainder?
if nargout > 1
    other.data = measurements.data;
    other.data(idx,:) = [];
    other.n = size(other.data, 1);
end

% do the work
measurements.data = measurements.data(idx, :);
measurements.n = size(measurements.data, 1);
