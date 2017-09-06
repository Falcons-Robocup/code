function kpi = fbt_kpi(settings, measurements, solutions, varargin)
% evaluate performance of solution(s)
% if a simulation scene is given, then compare against that reference
% TODO: support multiple solution structs, average (for monte-carlo analysis) 


% option parsing
options.scene = [];
[options, args] = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% count the number of balls detected as maximum of number of alive trackers
kpi.numballs = max(solutions.n);

% remove poor solutions at the start, where the solver is most sensitive due to only using a few measurements
tmin         = min(solutions.t) + settings.solver.bounce.age;
idx          = find(solutions.t >= tmin);
fn           = fieldnames(solutions);
for ifield = 1:numel(fn)
    arr      = getfield(solutions, fn{ifield});
    arr      = arr(idx);
    solutions = setfield(solutions, fn{ifield}, arr);
end

% apply filters
solutions    = fbt_solve_filter(solutions, '-filtertype', 'minmax');
solutions    = fbt_solve_filter(solutions, '-filtertype', 'mamsd');

% stability KPI's (r, vr), only tracker 1 for now
itracker     = 1;
idx          = find(solutions.id == itracker);
kpi.stab.pos = std(sqrt(solutions.x(idx).^2 + solutions.y(idx).^2 + solutions.z(idx).^2));
kpi.stab.vel = std(sqrt(solutions.vx(idx).^2 + solutions.vy(idx).^2 + solutions.vz(idx).^2));

% store filter result in kpi struct
kpi.filtered = solutions.filtered;

% compare result with simulated scene in case it is provided
if isstruct(options.scene)
    [kpi.errors, kpi.ballsim] = sub_errors(settings, options.scene, solutions);
end



% end of main, sub functions below

function [errors, ballsim] = sub_errors(settings, scene, solutions)
    % instead of evaluating ball position on solver time ticks,
    %      ballsim   = fbt_sim_ball(scene.ball, solutions.t);
    % we re-interpolate previously calculated scene, because otherwise the simulation does not make sense anymore (e.g. tstart functionality) 
    fn        = {'x', 'y', 'z', 'vx', 'vy', 'vz'};
    ballsim   = [];
    ballsim.t = solutions.t;
    errors    = [];
    for ifn = 1:numel(fn)
        intp    = interp1(scene.ballsim.t, getfield(scene.ballsim, fn{ifn}), solutions.t, 'linear', 'extrap');
        ballsim = setfield(ballsim, fn{ifn}, intp);
        err     = getfield(ballsim, fn{ifn}) - getfield(solutions, fn{ifn});
        errors  = setfield(errors, fn{ifn}, err);
    end

    