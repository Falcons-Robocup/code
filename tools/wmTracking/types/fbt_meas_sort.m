function measurements = fbt_meas_sort(measurements)
% merge measurements, sort in ascending time


% sort
t            = measurements.data(:, 3);
[~, idx]     = sort(t);
measurements = fbt_meas_reindex(measurements, idx);

