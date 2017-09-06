function measurements = fbt_meas_addcart(measurements)
% extend measurements struct with ball coordinates in FCS


% return if already done
idx_bx = 12;
if (size(measurements.data, 2) > idx_bx)
    return;
end

% calculate
xc    = measurements.data(:, 4);
yc    = measurements.data(:, 5);
zc    = measurements.data(:, 6);
phi   = measurements.data(:, 7);
az    = measurements.data(:, 8);
el    = measurements.data(:, 9);
r     = measurements.data(:, 10);
[bx, by, bz] = fct_ball2fcs(xc, yc, zc, phi, az, el, r);

% add columns
measurements.data = [measurements.data bx by bz];
