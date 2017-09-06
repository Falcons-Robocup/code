function result = fbt_sim_monte_carlo(scene, settings, errors, varargin)
% simulate N times, gather results


% defaults
options.N       = 10;
options.idxmask = [];
options.cbpre   = [];
options.cbpost  = [];

% parse options
[options, args] = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% for reproducability, hard-link to rand seed
for iseed = 1:options.N
    if ~isempty(options.cbpre)
        [scene2, settings2, errors2] = feval(options.cbpre, scene, settings, errors, iseed);
    else
        [scene2, settings2, errors2] = [scene, settings, errors];
    end
    data                   = fbt_sim_solve(scene2, settings2, errors2, iseed);
    if ~isempty(options.cbpost)
        data = feval(options.cbpost, data);
    end
    result.final.x(iseed)  = data.solution.merged.x(end);
    result.final.y(iseed)  = data.solution.merged.y(end);
    result.final.z(iseed)  = data.solution.merged.z(end);
    result.final.vx(iseed) = data.solution.merged.vx(end);
    result.final.vy(iseed) = data.solution.merged.vy(end);
    result.final.vz(iseed) = data.solution.merged.vz(end);
    % roll up and store KPI's 
    kpi                    = fbt_kpi(settings2, data.measurements, data.solution.merged, '-scene', data.scene);
    idxmask                = options.idxmask;
    if isempty(idxmask)
        idxmask            = 1:numel(kpi.errors.x);
    end
    result.kpi.errpos.x(iseed) = max(abs(kpi.errors.x(idxmask)));
    result.kpi.errpos.y(iseed) = max(abs(kpi.errors.y(idxmask)));
    result.kpi.errpos.z(iseed) = max(abs(kpi.errors.z(idxmask)));
    result.kpi.errvel.x(iseed) = max(abs(kpi.errors.vx(idxmask)));
    result.kpi.errvel.y(iseed) = max(abs(kpi.errors.vy(idxmask)));
    result.kpi.errvel.z(iseed) = max(abs(kpi.errors.vz(idxmask)));
    result.kpi.stab.pos(iseed) = kpi.stab.pos;
    result.kpi.stab.vel(iseed) = kpi.stab.vel;
end
