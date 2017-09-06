function [xo, yo, zo] = fct_rcs2fcs(xr, yr, phir, xi, yi, zi)
% convert coordinates in RCS to FCS
% inputs may be arrays (n*1)


% call generic transformation function with specific matrix constructor
[xo, yo, zo] = fct_transform(xr, yr, phir, xi, yi, zi, @fct_mat_rcs2fcs);

