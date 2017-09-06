function solution_out = fbt_sol_filter(solution_in, varargin)
% filter a solution struct


% default filter criteria
% (factored out for reuse in callers, to avoid costly option parsing)
options       = fbt_filter_opts;
% parse options
[options, args] = getopts(options, varargin{:});
assert(numel(args) == 0); % don't know what to do with unparsed arguments

% time is already unpacked.. so we need to iterate
tcurr = -1;
tol = 1e-4;
idxkeep = [];
ncurr = 0;
for idx = 1:numel(solution_in.t)
    x = solution_in.x(idx);
    y = solution_in.y(idx);
    z = solution_in.z(idx);
    conf = solution_in.conf(idx);
    if abs(solution_in.t(idx) - tcurr) > tol
        % re-init
        tcurr = solution_in.t(idx);
        ncurr = 0;
        if sub_check(x, y, z, conf, ncurr, options)
            idxkeep(end+1) = idx;
            ncurr = 1;
        end
    else % continue
        % add if criteria are met
        if sub_check(x, y, z, conf, ncurr, options)
            idxkeep(end+1) = idx;
            ncurr = ncurr + 1;
        end
    end
end

% reindex
solution_out.t = solution_in.t(idxkeep);
solution_out.id = solution_in.id(idxkeep);
solution_out.x = solution_in.x(idxkeep);
solution_out.y = solution_in.y(idxkeep);
solution_out.z = solution_in.z(idxkeep);
solution_out.vx = solution_in.vx(idxkeep);
solution_out.vy = solution_in.vy(idxkeep);
solution_out.vz = solution_in.vz(idxkeep);
solution_out.conf = solution_in.conf(idxkeep);
solution_out.n = solution_in.n(idxkeep);



function ok = sub_check(x, y, z, conf, ncurr, options)
    ok = 0;
    if ((x >= options.xmin) & (x <= options.xmax) & (y >= options.ymin) & (y <= options.ymax) & (z >= options.zmin) & (z <= options.zmax))
        if options.confmin <= conf
            if ncurr + 1 <= options.maxballs
                ok = 1;
            end
        end
    end
