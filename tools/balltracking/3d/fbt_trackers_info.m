function fbt_trackers_info(trackers, tcurr, settings, showheading, idxshow)
% show some info on trackers, one line each



% settings
options.showbad = (settings.solver.verbosity >= 4);
options.showconfdetails = (settings.solver.verbosity >= 3);

% interface is a bit messy...
numtrackers = numel(trackers);
if nargin < 5
    idxshow = 1:numtrackers;
end
if nargin < 4
    showheading = 1;
end

% count good balls
numballs = 0;
for itracker = 1:numel(trackers)
    numballs = numballs + trackers{itracker}.good;
end

% rows
s = '';
for itracker = idxshow
    tr = trackers{itracker};
    tmax = max(tr.measurements.data(:,3));
    assert(tmax <= tcurr);
    if tr.good || options.showbad
        s = [s sprintf('%16.6f %3d %2d %4d %4.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %5.1f %5d', ...
            tcurr, numtrackers, numballs, ...
            tr.id, tr.conf, ...
            tr.solution.x, tr.solution.y, tr.solution.z, ...
            tr.solution.vx, tr.solution.vy, tr.solution.vz, ...
            tcurr - tr.t0, ...
            tr.measurements.n)];
        if options.showconfdetails
            [~, conf_details] = fbt_tracker_confidence(tr, tcurr, settings);
            s = [s sprintf('%5d %4d %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f', ...
                conf_details.numcams, conf_details.withomni, ...
                conf_details.camscore, conf_details.agescore, conf_details.measscore, conf_details.freshscore, ...
                conf_details.zscore, conf_details.vscore, ...
                conf_details.fitscore, conf_details.fitquality)];
        end
        s = [s sprintf('\n')];
    end
    if conf_details.fitquality < 0.01
        % suspiciously good fit, simulation??
        1;
    end
end

% display heading
if showheading
    hs = '';
    hs = [hs sprintf('#%15s %3s %2s %4s %4s %6s %6s %6s %6s %6s %6s %5s %5s', 'time', '#tr', '#b', 'id', 'conf', 'solx', 'soly', 'solz', 'solvx', 'solvy', 'solvz', 'age', '#meas')];
    if options.showconfdetails
        hs = [hs sprintf(' %4s %4s %4s %4s %4s %4s %4s %4s %4s %4s', 'ncam', 'omni', 'qcam', 'qage', 'qm', 'qfr', 'qz', 'qv', 'qfit', 'res')];
    end
    disp(hs);
end

% display rows
if numel(s)
    fprintf(1, s);
else
    % no data, show a line for this solver tick anyway
    fprintf(1, sprintf('%16.6f %3d\n', tcurr, numtrackers));
end
