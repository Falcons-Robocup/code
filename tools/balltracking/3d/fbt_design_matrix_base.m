function D = fbt_design_matrix_base(measurement)
% create the design matrix D for a single measurement


% unpack measurements array
xc    = measurement.data(:, 4);
yc    = measurement.data(:, 5);
zc    = measurement.data(:, 6);
phi   = measurement.data(:, 7);
az    = measurement.data(:, 8);
el    = measurement.data(:, 9);

% number of measurements
n     = measurement.n;
assert(n == 1);

% construct matrix
% solution is in FCS -> convert to RCS
D1     = fct_mat_fcs2rcs(xc, yc, phi);
% convert RCS to CCS by taking camera mounting height into account
D1     = D1 * fct_translate(0, 0, -zc);
% rotate away both measurement angles
D2     = fct_xrotate(-el) * fct_zrotate(-az);
D      = (D2 * D1)';

