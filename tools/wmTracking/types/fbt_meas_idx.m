function idx = fbt_meas_idx(s)
% get index of certain column in measurements.data

[~, idx] = ismember(s, fbt_meas_fn);
