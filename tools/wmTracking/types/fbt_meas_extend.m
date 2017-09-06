function measurements = fbt_meas_extend(measurements)
% extend measurements struct with ball coordinates in FCS etc.


% return if already done
idx_bx = fbt_col_idx('bx', measurements);
if (size(measurements.data, 2) > idx_bx)
    return;
end

% calculate
xc    = measurements.data(:, fbt_col_idx('cx', measurements));
yc    = measurements.data(:, fbt_col_idx('cy', measurements));
zc    = measurements.data(:, fbt_col_idx('cz', measurements));
phi   = measurements.data(:, fbt_col_idx('cphi', measurements));
az    = measurements.data(:, fbt_col_idx('az', measurements));
el    = measurements.data(:, fbt_col_idx('el', measurements));
r     = measurements.data(:, fbt_col_idx('r', measurements));
[bx, by, bz] = fct_ball2fcs(xc, yc, zc, phi, az, el, r);

% add columns
measurements.data = [measurements.data bx by bz];

% add t0
measurements.t0 = min(measurements.data(:, fbt_col_idx('t', measurements)));
