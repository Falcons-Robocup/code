function M = fct_mat_rcs2fcs(xr, yr, phir)
% return 4D transformation matrix to convert coordinates in RCS to FCS
% inputs must be scalar


% check vectors all have the same length
n      = numel(xr);

% construct matrix: first rotate, then translate
M      = fct_translate(xr, yr, 0) * fct_zrotate(phir - 0.5*pi);

