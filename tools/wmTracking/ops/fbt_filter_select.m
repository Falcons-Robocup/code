function data_out = fbt_filter_select(data_in, col, values)
% filter data based on certain values in 1 column

data_out = data_in;
col_idx  = fbt_col_idx(col, data_in);
id       = find(ismember(data_out.data(:, col_idx), values));
data_out = fbt_data_reindex(data_in, id);
