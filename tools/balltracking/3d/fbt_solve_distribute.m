function trackers = fbt_solve_distribute(settings, trackers, measurements)
% distribute measurements over trackers


% sanity checks
if measurements.n == 0
    %warning('no measurements to distribute?!');
    return;
end

% data dump?
if settings.solver.verbosity >= 5
    disp(fbt_meas_2str(measurements, true));
end

% try to match measurements with existing trackers
for imeas = 1:measurements.n
    assigned = 0;
    % NOTE: in case a measurement could be matched with multiple trackers, the first one gets it!
    thismeas = fbt_meas_reindex(measurements, imeas);
    for itracker = 1:numel(trackers)
        tr = trackers{itracker};
        [~, tr, n] = sub_assign_meas(settings, thismeas, tr);
        if (n == 1)
            if settings.solver.verbosity >= 5
                disp(sprintf('assigned %d measurements to existing tracker %d at (%6.2f, %6.2f)', n, itracker, tr.solution.x, tr.solution.y));
            end
            trackers{itracker} = tr;
            assigned = 1;
            break;
        end
    end
    % create new tracker?
    if ~assigned
        tr       = fbt_tracker(thismeas);
        if settings.solver.verbosity >= 5
            disp(sprintf('creating a new tracker %d at (%6.2f, %6.2f)', tr.id, tr.solution.x, tr.solution.y));
        end
        tr.t0    = thismeas.data(1, 3); % store timestamp for confidence heuristic
        trackers{end+1} = tr;
    end
end



% end of main, sub functions below

function [mo, tr, n] = sub_assign_meas(settings, mi, tr)
    % match on proximity (TODO: also use speed, angles?)
    % TODO: we match on (x,y,z), but the configuration parameter is still named 'xytol'
    bx       = mi.data(:, 12);
    by       = mi.data(:, 13);
    bz       = mi.data(:, 14);
    dx       = bx - tr.solution.x;
    dy       = by - tr.solution.y;
    dz       = bz - tr.solution.z;
    dist     = sqrt(sum([dx dy dz].^2));
    idx      = find(dist < settings.solver.tracker.xytol);
    n        = numel(idx);
    % split the dataset in KEEP and REMAINDER
    [m_add, mo] = fbt_meas_reindex(mi, idx);
    tr.measurements = fbt_meas_merge(tr.measurements, m_add);
