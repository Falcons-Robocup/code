function [trackers, numballs, idx_good] = fbt_solve_confidence(settings, trackers, tcurr)
% determine confidence for each tracker
% confidence is a value between 0.0 (bad) and 1.0 (good)


% TODO: roadmap: use line of sight criterion: if a robot could have seen
% the ball, but didn't, this should decrease the confidence (idea from Tim 20161027) 

% TODO: note: with current design, in principle every robot should come to the same conclusion
% aside from WIFI packet loss & latency
% in visualizer, inconsistencies must be shown, if present
% in future, we might also use robot-dependent confidence sub-scores, e.g. distance to ball, has it 
% seen the ball itself. 

% calculate confidence for each tracker
for itracker = 1:numel(trackers)
    % NOTE: floor clipping might be applied
    trackers{itracker} = fbt_tracker_confidence(trackers{itracker}, tcurr, settings);
end

% scale confidence with maximum, to make sure all values in [0.0, 1.0]
if settings.solver.conf.autoscale
    confMax = 0;
    for itracker = 1:numel(trackers)
        confMax = max(confMax, trackers{itracker}.conf);
    end
    if confMax > 0
        for itracker = 1:numel(trackers)
            trackers{itracker}.conf = trackers{itracker}.conf / confMax;
        end
    end
end

% flag the trackers for which confidence is good enough
numballs = 0;
for itracker = 1:numel(trackers)
    trackers{itracker}.good = (trackers{itracker}.conf > settings.solver.conf.goodlim);
    if trackers{itracker}.good
        numballs = 1 + numballs;
    end
end

% sort trackers by decreasing confidence
conf     = [];
for itracker = 1:numel(trackers)
    conf(end+1) = trackers{itracker}.conf;
end
reindex  = 1:numel(trackers);
[~, idx] = sort(conf);
reindex  = reindex(idx(end:-1:1));
idx_good = reindex(1:numballs);

% apply the 'maybe' criterium: only when no 'good' balls are present,
% we pick M best balls below the 'good' confidence threshold
% NOTE: in visualizer, we should draw 'maybe' balls orange or something
if ~numballs
    idx_good = find(conf > settings.solver.conf.maybelim);
    if numel(idx_good) > settings.solver.conf.nummaybe
        idx_good = idx_good(1:settings.solver.conf.nummaybe);
    end
    for itracker = idx_good
        trackers{itracker}.good = true;
    end
    numballs = numel(idx_good);
end


