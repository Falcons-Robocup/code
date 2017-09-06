function [xo, yo, zo] = fct_fcs2rcs(xr, yr, phir, xi, yi, zi)
% convert coordinates in FCS to RCS
% inputs may be arrays (n*1)


% call generic transformation function with specific matrix constructor
[xo, yo, zo] = fct_transform(xr, yr, phir, xi, yi, zi, @fct_mat_fcs2rcs);

