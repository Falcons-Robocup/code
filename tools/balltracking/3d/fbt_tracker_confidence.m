function [tracker, conf_details] = fbt_tracker_confidence(tracker, tcurr, settings)
% determine confidence for given tracker
% confidence is a value between 0.0 (bad) and 1.0 (good)


% TODO: roadmap: use line of sight criterion: if a robot could have seen
% the ball, but didn't, this should decrease the confidence (idea from Tim 20161027) 

% baseScore is a value in [0.0, 1.0]: TODO update, WIP
%    numberOfCameras * (1+withOmniVision) * 0.1
%    so if omnivision is contributing, then baseScore > 0.5
%    so if omnivision is not contributing, then baseScore < 0.5
%    numberOfCameras > 5 has no effect, so value is clipped at 5
% ageScore is also a value in [0.0, 1.0]
% resulting confidence (unscaled) = ageScore * baseScore

conf_details.scale      = settings.solver.conf.numcams * (1 + settings.solver.conf.omnipref);
conf_details.withomni   = any(tracker.measurements.data(:,2) == 1);
conf_details.numcams    = numel(unique(sort(tracker.measurements.data(:,1))));
conf_details.camscore   = min(conf_details.numcams, settings.solver.conf.numcams) * (1 + (conf_details.withomni && settings.solver.conf.omnipref)) / conf_details.scale;
conf_details.measscore  = min(tracker.measurements.n, settings.solver.conf.measlim) / settings.solver.conf.measlim;
tlast                   = max(tracker.measurements.data(:,3));
age                     = tlast - tracker.t0;
conf_details.agescore   = min(1.0, age / settings.solver.conf.agelim);
freshness               = tcurr - tlast;
conf_details.freshscore = 1.0 - max(0.0, (freshness - settings.solver.conf.freshlim) / (settings.solver.tracker.timeout - settings.solver.conf.freshlim));
conf_details.fitquality = tracker.solution.quality;
conf_details.fitscore   = 1.0 - min(max((tracker.solution.quality - settings.solver.conf.fitlim1) / (settings.solver.conf.fitlim2 - settings.solver.conf.fitlim1), 0.0), 1.0);
% TODO: use erfc instead of linear scoring, especially for fit residuals
conf_details.zscore     = 1.0 - min(max((tracker.solution.z - settings.solver.conf.zlim1) / (settings.solver.conf.zlim2 - settings.solver.conf.zlim1), 0.0), 1.0);
velocity                = sqrt(sum([tracker.solution.vx tracker.solution.vy tracker.solution.vz].^2));
conf_details.vscore     = 1.0 - min(max((velocity - settings.solver.conf.vlim1) / (settings.solver.conf.vlim2 - settings.solver.conf.vlim1), 0.0), 1.0);
assert(conf_details.agescore >= 0.0);
assert(conf_details.camscore >= 0.0);
assert(conf_details.measscore >= 0.0);
assert(conf_details.freshscore >= 0.0);
assert(conf_details.fitscore >= 0.0);
assert(conf_details.zscore >= 0.0);
assert(conf_details.vscore >= 0.0);
assert(conf_details.agescore <= 1.0);
assert(conf_details.camscore <= 1.0);
assert(conf_details.measscore <= 1.0);
assert(conf_details.freshscore <= 1.0);
assert(conf_details.fitscore <= 1.0);
assert(conf_details.zscore <= 1.0);
assert(conf_details.vscore <= 1.0);
conf_details.total      = conf_details.agescore * conf_details.camscore * conf_details.measscore * conf_details.freshscore * conf_details.fitscore * conf_details.zscore * conf_details.vscore;

% floor clipping?
if (tracker.solution.z < 0) && settings.solver.tracker.floorclip
    if (tracker.solution.z < -0.1) && (conf_details.total > 0.3)
        warning('clipping ball result below floor (z=%6.2f)', tracker.solution.z);
        conf_details.total = 0.8 * conf_details.total; % poor man penalty
    end
    tracker.solution.z  = 0.0;
end
tracker.conf            = conf_details.total;
