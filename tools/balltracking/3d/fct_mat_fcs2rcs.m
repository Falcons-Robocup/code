function M = fct_mat_fcs2rcs(xr, yr, phir)
% return 4D transformation matrix to convert coordinates in FCS to RCS
% inputs must be scalar


% check vectors all have the same length
n      = numel(xr);

% construct matrix: first translate, then rotate
M      = fct_zrotate(-phir + 0.5*pi) * fct_translate(-xr, -yr, 0);

