function tracker = fbt_tracker(measurement, varargin)
% create a tracker object


% persistent counter for tracker id, reset at the beginning of fbt_solve_sequence
persistent id;
if nargin > 1
    id      = 0;
    tracker = [];
    return
end

% create based on given measurement, measurement assigner and solver will kick in later
id                   = 1 + id;
tracker.measurements = measurement;
tracker.id           = id;
tracker.good         = false;
tracker.solution.x   = measurement.data(1, fbt_meas_idx('bx'));
tracker.solution.y   = measurement.data(1, fbt_meas_idx('by'));
tracker.solution.z   = measurement.data(1, fbt_meas_idx('bz'));
tracker.blacklist    = 0;
