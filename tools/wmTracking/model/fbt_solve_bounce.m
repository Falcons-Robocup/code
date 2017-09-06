function [solution, bounce_found] = fbt_solve_bounce(settings, measurements, tcurr)
% solve by iteratively calling fit, determining the moment of bounce
% literature    : https://en.wikipedia.org/wiki/Segmented_regression
% our take on it: just fit in two fully separated parts
% roadmap       : fit in one go, requiring the line segments to be connected
% heuristics    : (TODO) use fit residuals... ?


% clip measurements according to solver timeout
filter_opts    = fbt_filter_opts;
filter_opts.tmin = tcurr - settings.solver.tracker.timeout;
measurements   = fbt_meas_filter(measurements, filter_opts);
tmin           = min(measurements.data(:, 3));
tmax           = max(measurements.data(:, 3));

% construct bounce breakpoints (last is no-breakpoint)
dt             = settings.solver.bounce.dt;
tbounce        = (tcurr - dt) : -dt : (tmin + dt);
tspread        = tmax - tmin;
if (tspread < 0.4 * settings.solver.bounce.age) || (tcurr - settings.solver.bounce.age < 0)
    % little data (spread), do not apply bounce detection
    tbounce    = [];
end

% run solver without bounce breakpoint
[solution, res] = sub_solve(settings, measurements, tcurr);
quality        = solution.quality;

% step over bounce breakpoints
bounce_found   = 0;
bounce_t       = 0;
delta_vel      = 0;
%assert(numel(tbounce) == 0); % for now we are working without speed detection
for ibounce = 1:numel(tbounce)
    tb       = tbounce(ibounce);
    % split measurements
    filter_opts.tmax = tb;
    [m1, m2] = fbt_meas_filter(measurements, '-tmax', tb); % TODO filter_opts
    % only proceed if both segments have enough measurements
    if min(m1.n, m2.n) < settings.solver.bounce.minmeas
        continue;
    end
    % left side (oldest)
    [s1, r1] = sub_solve(settings, m1, tb);
    % right side (newest)
    [s2, r2] = sub_solve(settings, m2, tcurr);
    % determine quality
    q        = fbt_solve_fit_quality([r1; r2]);
    if (q < quality)
        bs   = sqrt(sum([s2.vx-s1.vx, s2.vy-s1.vy, s2.vz-s1.vz].^2));
        if (bs > settings.solver.bounce.mindv)
            % take over solution
            solution     = s2;
            solution.quality = q;
            bounce_found = 1;
            bounce_t     = tb; % TODO: store this in solution, put on output interface (diagnostics)
            bounce_size  = bs;
            % debug
            %close all ; figure; fbt_plot(m1, s1); fbt_meas_plot(m2); fbt_sol_plot(s2)
            %close all ; fbt_gui(measurements);
        end
    end
end

% verbose
if 0*bounce_found
    warning('bounce detected at tb=%.2f (tcurr=%.2f), |size|=%.1f', bounce_t, tcurr, bounce_size);
end


% end of main, sub functions below

function [solution, residuals] = sub_solve(settings, measurements, tcurr)
    % relay to core solver
    [solution, residuals] = fbt_solve_fit(settings, measurements, tcurr);
    
