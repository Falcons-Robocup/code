function data_out = fbt_filter_robots(data_in, robot_ids)
% filter measurements based on robot id

data_out = data_in;
col_idx  = fbt_col_idx('id', data_in);
id       = find(ismember(data_out.data(:, col_idx), robot_ids));
data_out = fbt_data_reindex(data_in, id);
