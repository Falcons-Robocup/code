function solutions = fbt_solve_sequence(settings, measurements, varargin)
% solve by discriminating ball trackers and feeding measurements over time
% this is computationally intensive...


% initialize
trackers     = {};
n            = measurements.n;
t            = measurements.data(:, fbt_meas_idx('t'));
fbt_tracker([], 0); % reset tracker id
solutions    = [];
sol_fn       = {'t', 'id', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'conf', 'n'};
for ifn = 1:numel(sol_fn)
    solutions = setfield(solutions, sol_fn{ifn}, []);
end

% option parsing
options.pc   = []; % progress callback
[options]    = getopts(options, varargin{:});
doprogress   = isa(options.pc, 'function_handle');

% make sure data is sorted and extended with (bx, by, bz)
measurements = fbt_meas_sort(measurements);
measurements = fbt_meas_addcart(measurements);

% also make sure design matrix cache is present
measurements = fbt_meas_add_dmcache(measurements);

% pre-filter measurements which do not meet certain criteria
% * out of bounds
% * confidence too low
[measurements, removed] = fbt_meas_filter(measurements);
t            = measurements.data(:, fbt_meas_idx('t'));
if settings.solver.verbosity >= 1
    disp(sprintf('# fbt_solve_sequence: %d measurements prefiltered (%.1f%%)', removed.n, 100.0 * removed.n / measurements.n));
    if settings.solver.verbosity >= 4
        disp(fbt_meas_2str(removed, true));
    end
end

% setup simulation time frame
tmin         = min(t);
tmax         = max(t);
tframe       = tmax - tmin;
dt           = 1.0 / settings.solver.ana.frequency;
lastprogress = -1;
itermax      = tframe / dt;

% run through time, simulate measurements 
tcurr        = tmin;
tprev        = 0;
warned       = 0;
iter         = 0;
while tcurr + dt < tmax
    % update time
    iter     = 1 + iter;
    tcurr    = tmin + iter * dt;
    % progress bar update
    if doprogress
        currprogress = round(iter * 100.0 / itermax);
        if currprogress > lastprogress
            feval(options.pc, 0.01 * currprogress);
            lastprogress = currprogress;
        end
    end
    % display some info?
    if settings.solver.verbosity >= 4
        disp(sprintf('solve_sequence iteration %d, #trackers=%d, t=%6.2f (%s)', iter, numel(trackers), tcurr-tmin, fbt_time_float2str(tcurr)));
    end
    % distribute all measurements in [tprev, tcurr] to trackers (this may be slow...)
    idx      = find((t > tprev) & (t <= tcurr));
    trackers = fbt_solve_distribute(settings, trackers, fbt_meas_reindex(measurements, idx));
    % run solver on each tracker; delete timed-out trackers
    trackers_orig = trackers;
    trackers = fbt_solve_update(settings, trackers, tcurr, tmin);
    % confidence heuristic
    [trackers, numballs, idx_good] = fbt_solve_confidence(settings, trackers, tcurr);
    % TODO: apply some blacklist criteria for false positive, e.g. floating objects
    %trackers = fbt_solve_blacklist(settings, trackers);
    % TODO: apply a smoothener to good solutions? first evaluate performance (e.g. MA/MSD)

    % check number of balls
    assert(numel(trackers) < 100); % highly unlikely we would have this many trackers, probably SW bug
    if numballs > settings.solver.tracker.numwarn
        if numballs > warned
            warning(sprintf('tracking %d balls (%d trackers) at t=%.3f (%s)', numballs, numel(trackers), tcurr - tmin, fbt_time_float2str(tcurr)));
            warned = numballs;
        end
    end
    % store the solution per tracker in output
    if numballs == 0
        % no good balls, but make an entry anyway
        solutions.t(end+1,1)    = tcurr;
        solutions.id(end+1,1)   = 0;
        solutions.x(end+1,1)    = 0;
        solutions.y(end+1,1)    = 0;
        solutions.z(end+1,1)    = 0;
        solutions.vx(end+1,1)   = 0;
        solutions.vy(end+1,1)   = 0;
        solutions.vz(end+1,1)   = 0;
        solutions.conf(end+1,1) = 0;
        solutions.n(end+1,1)    = numballs;
    else
        for itracker = idx_good
            if isfield(trackers{itracker}.solution, 'x') % only at the very start, solver may not have been called yet
                solutions.t(end+1,1)    = tcurr;
                solutions.id(end+1,1)   = trackers{itracker}.id;
                solutions.x(end+1,1)    = trackers{itracker}.solution.x;
                solutions.y(end+1,1)    = trackers{itracker}.solution.y;
                solutions.z(end+1,1)    = trackers{itracker}.solution.z;
                solutions.vx(end+1,1)   = trackers{itracker}.solution.vx;
                solutions.vy(end+1,1)   = trackers{itracker}.solution.vy;
                solutions.vz(end+1,1)   = trackers{itracker}.solution.vz;
                solutions.conf(end+1,1) = trackers{itracker}.conf;
                solutions.n(end+1,1)    = numballs;
            end
        end
    end
    % prepare for next iteration
    tprev = tcurr;
    % display statistics?
    if settings.solver.verbosity >= 2
        showheading = (iter == 1);
        if settings.solver.verbosity >= 4
            % TODO sort trackers by decreasing confidence
            fbt_trackers_info(trackers, tcurr, settings, showheading);
        else
            fbt_trackers_info(trackers, tcurr, settings, showheading, idx_good);
        end
        1;
    end
end


