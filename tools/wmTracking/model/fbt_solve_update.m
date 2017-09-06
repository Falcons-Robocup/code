function trackers = fbt_solve_update(settings, trackers, tcurr, t0)
% update each tracker by calling solver and cleaning up old data


% arguments
if nargin < 4
    t0 = 0;
end

% statistics
stats.before = numel(trackers);
stats.after  = 0; % to be filled later
stats.removed = 0; % to be filled later
stats.updated = 0; % to be filled later

% remove old trackers
told         = tcurr - settings.solver.tracker.timeout;
tlim         = tcurr - 1.0 / settings.solver.ana.frequency;
iremove      = [];
idx_t        = fbt_meas_idx('t');
for itracker = 1:numel(trackers)
    if trackers{itracker}.measurements.n == 0
        iremove(end+1) = itracker;
    else
        if max(trackers{itracker}.measurements.data(:, idx_t)) < told
            iremove(end+1) = itracker;
        end
    end
end
trackers(iremove) = [];

% check if tracker solution needs an update due to new measurements
needupdate   = [];
filter_opts  = fbt_filter_opts;
for itracker = 1:numel(trackers)
    tr       = trackers{itracker};
    t        = tr.measurements.data(:, idx_t);
    assert(max(t) <= tcurr);
    %ALWAYS if max(t) > tlim
        % clear out old data
        filter_opts.tmin = told;
        tr.measurements = fbt_meas_filter(tr.measurements, filter_opts);
        assert(tr.measurements.n > 0); % how can this happen?
        if tr.measurements.n
            needupdate(end+1) = itracker;
        end
        trackers{itracker} = tr;
    %end
end

% call solver
for itracker = 1:numel(trackers) % needupdate(:)'
    [sol] = fbt_solve_bounce(settings, trackers{itracker}.measurements, tcurr);
    trackers{itracker}.solution = sol;
end

% statistics
stats.after = numel(trackers);
stats.removed = stats.before - stats.after;
stats.updated = numel(needupdate);
if settings.solver.verbosity >= 4
    disp(sprintf('   %3d trackers (%d cleaned, %d updated)', stats.after, stats.removed, stats.updated));
end
