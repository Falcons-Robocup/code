function s = fbt_meas_2str(measurements, withxyz)
% convert measurements to string
% see also: fbt_time_float2str

if nargin == 1
    withxyz = 0;
end

% convert
nmeas = measurements.n;
s     = '';
id    = measurements.data(:, fbt_meas_idx('id'));
t     = measurements.data(:, fbt_meas_idx('t'));
cam   = measurements.data(:, fbt_meas_idx('cam'));
xc    = measurements.data(:, fbt_meas_idx('xc'));
yc    = measurements.data(:, fbt_meas_idx('yc'));
zc    = measurements.data(:, fbt_meas_idx('zc'));
phi   = measurements.data(:, fbt_meas_idx('phi'));
az    = measurements.data(:, fbt_meas_idx('az'));
el    = measurements.data(:, fbt_meas_idx('el'));
r     = measurements.data(:, fbt_meas_idx('r'));
conf  = measurements.data(:, fbt_meas_idx('conf'));
if withxyz
    bx = measurements.data(:, fbt_meas_idx('bx'));
    by = measurements.data(:, fbt_meas_idx('by'));
    bz = measurements.data(:, fbt_meas_idx('bz'));
end
for imeas = 1:nmeas
    si = sprintf('%2d  %d  %16.6f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f', ...
        id(imeas), cam(imeas), t(imeas), ... % robotId, camId, timestamp
        xc(imeas), yc(imeas), zc(imeas), phi(imeas), ... % camera x, y, z, robot phi
        az(imeas), el(imeas), r(imeas), conf(imeas));  % ball az, el, r, conf
    if withxyz
        si = [si sprintf('  %9.5f  %9.5f  %9.5f', bx(imeas), by(imeas), bz(imeas))];
    end
    si = [si sprintf('\n')];
    s = [s si];
end

% fix numerical zeros (which are annoying in diff-based testing)
s     = regexprep(s, ' -0.(0+\s)', '  0.$1');


