function idx = fbt_col_idx(s, data)
% get index of certain column in data struct

[~, idx] = ismember(s, data.columns);
