function measurements = fbt_meas(n)
% create a measurements struct


if nargin == 0
    n = 0;
end

% columns
fn = {'id', 'cam', 't', 'cx', 'cy', 'cz', 'cphi', 'az', 'el', 'r', 'conf', 'bx', 'by', 'bz'};
measurements.columns = fn;
measurements.data = zeros(n, numel(fn)-3);
measurements.n = n;

% NOTE: by default, the metadata columns are empty (bx,by,bz)
% until explicitly filled by fbt_meas_extend
