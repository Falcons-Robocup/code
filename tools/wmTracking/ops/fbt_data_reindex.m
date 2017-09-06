function data_out = fbt_data_reindex(data_in, idx)
% filter measurements based on row index

data_out      = data_in;
data_out.data = data_out.data(idx, :);
data_out.n    = numel(idx);

