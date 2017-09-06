function measurements = fbt_meas_add_dmcache(measurements)
% extend measurements struct with cached design matrix


% return if already done
idx = 15;
if (size(measurements.data, 2) > idx)
    return;
end

% number of measurements
n     = measurements.n;

% calculate
oidx  = 15:30;
for imeas = 1:n
    D = fbt_design_matrix_base(fbt_meas_reindex(measurements, imeas));
    % store in measurements
    measurements.data(imeas, oidx) = D(:);
end
