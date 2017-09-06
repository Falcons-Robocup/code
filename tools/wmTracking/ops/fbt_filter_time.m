function data_out = fbt_filter_time(data_in, tmin, tmax)
% filter data based on time

col_idx  = fbt_col_idx('t', data_in);
t        = data_in.data(:, col_idx);
id       = find((t >= tmin) & (t <= tmax));
data_out = fbt_data_reindex(data_in, id);
