function simball = fbt_sim_ball(ball, t, settings)
% simulate ball movement in a 'exact' way based on given initial position+speed, on given timestamps
% input struct 'settings' is optional
% features: gravity, simple friction, bouncing on floor and 1 virtual wall (not other robots), stationary-until period
% NOTE: noise / errors are added in fbt_sim_robot
% NOTE: this simulation assumes ball is simply a point (no mass, no radius)



% unpack settings 
if nargin < 3
    settings = fbt_settings;
end
gravity      = settings.sim.gravity;
dragfactor   = (1.0 - settings.sim.ball.drag);
bouncefactor = (1.0 - settings.sim.ball.bouncedp);
walls        = {{'z', 0}, {'x', -8}}; % z=0 is the floor; x=-8 is handy for some fast-response test cases

% initialize
t          = t(:); % make sure t is column vector
n          = numel(t);
simball.x  = ball.x;
simball.y  = ball.y;
simball.z  = ball.z;
simball.vx = ball.vx;
simball.vy = ball.vy;
simball.vz = ball.vz;
t0         = 0;
if isfield(ball, 'tstart')
    t0     = ball.tstart;
end

% simulate 
for it = 2:n
    % handle stationary-until period
    if t(it-1) < t0
        if (t(it) >= t0)
            simball.vx(it-1) = ball.vx;
            simball.vy(it-1) = ball.vy;
            simball.vz(it-1) = ball.vz;
        else
            simball.vx(it-1) = 0;
            simball.vy(it-1) = 0;
            simball.vz(it-1) = 0;
        end
    end
    % update position based on previous position and previous speed
    dt = t(it) - t(it-1);
    simball.x(it,1) = simball.x(it-1) + dt * simball.vx(it-1);
    simball.y(it,1) = simball.y(it-1) + dt * simball.vy(it-1);
    simball.z(it,1) = simball.z(it-1) + dt * simball.vz(it-1);
    % update speed for next iteration
    simball.vx(it,1) = simball.vx(it-1) * dragfactor;
    simball.vy(it,1) = simball.vy(it-1) * dragfactor;
    simball.vz(it,1) = simball.vz(it-1) * dragfactor + dt * gravity;
    % simulate bouncing
    for iwall = 1:numel(walls)
        coord     = walls{iwall}{1};
        threshold = walls{iwall}{2};
        value     = simball.(coord)(it);
        if value < threshold
            % bounce! mirror and negate speed
            newvalue = 2 * threshold - value;
            simball.(coord)(it,1) = newvalue;
            simball.(['v' coord])(it,1) = -1.0 * bouncefactor * simball.(['v' coord])(it);
        end
    end
end
simball.t = t;
