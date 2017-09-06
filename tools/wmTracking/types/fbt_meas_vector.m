function b = fbt_meas_vector(settings, measurements)
% create measurement vector, used in fbt_solve


% number of measurements
n     = measurements.n;

% construct vector
b     = [];
r     = measurements.data(:, 10);
for imeas = 1:n
    % add fourth dummy data point with value 1
    % set azimuth and elevation to zero because these have been transformed away in the design matrix
    % in RCS, y is the distance, which is the second component
    v = [0; r(imeas); 0; 1];
    b = [b; v];
end
