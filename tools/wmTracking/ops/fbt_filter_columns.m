function data_out = fbt_filter_columns(data_in, columns)
% filter data based on columns

col_idx     = [];
for icol = 1:numel(columns)
    col_idx = [col_idx fbt_col_idx(columns{icol}, data_in)];
end
data_out    = data_in;
data_out.data = data_out.data(:, col_idx);
data_out.columns = columns;
