function settings = fbt_settings(v)


% optional factor: default 1 (physics simulation), use 0 to disable
if nargin == 0
    v = 1;
end

% solver settings - general
settings.solver.verbosity         =  1;      % [1] 0 is quiet, higher is more detail
% solver settings - core fit
settings.solver.core.weight       =  0.1;    % [1] relative depth weights
settings.solver.core.speed        =  1;      % [bool] fit with speed - on robot, we always try to fit with speed
settings.solver.core.minvdt       =  0.05;   % [s] minimum measurement spread required for speed fitting, otherwise fallback to position-only
% solver settings - bounce detection
settings.solver.bounce.dt         =  0.10;   % [s] bounce detection resolution
settings.solver.bounce.age        =  0.50;   % [s] how far to look back in time in solver
settings.solver.bounce.minmeas    =  5;      % [1] minimum number of measurements per segment
settings.solver.bounce.mindv      =  6;      % [m/s] minimum speed change for bounce detection
% solver settings - object tracker
settings.solver.tracker.timeout   =  2.00;   % [s] when to discard measurements
settings.solver.tracker.xytol     =  2.5;    % [m] measurement grouping criterion
settings.solver.tracker.floorclip =  1;      % [bool] clip measurements with z<0 to z=0
settings.solver.tracker.numwarn   =  1;      % [1] warn in case more than this amount of balls are identified
% solver settings - confidence heuristics
settings.solver.conf.numcams      =  3;      % [1] required number of involved cameras for full confidence
settings.solver.conf.agelim       =  5;      % [s] required age of tracker for full confidence (if tracker starts at t=1, tcurr=4, then age=3)
settings.solver.conf.freshlim     =  0.0;    % [s] required freshness of tracker for full confidence, see also tracker timeout
settings.solver.conf.omnipref     =  1;      % [bool] omnivision required for full confidence
settings.solver.conf.measlim      = 20;      % [1] required number of measurements for full confidence
settings.solver.conf.zlim1        =  1;      % [m] threshold for fitted z result - we consider values lower than this threshold good
settings.solver.conf.zlim2        =  5;      % [m] threshold for fitted z result - we consider values higher than this threshold bad
settings.solver.conf.vlim1        =  2;      % [m/s] speed (vx,vy,vz) threshold - we consider values lower than this threshold good
settings.solver.conf.vlim2        = 20;      % [m/s] speed (vx,vy,vz) threshold - we consider values higher than this threshold bad
settings.solver.conf.fitlim1      =  0.20;   % [1] numerical fit residue threshold - we consider values lower than this threshold good
settings.solver.conf.fitlim2      =  0.60;   % [1] numerical fit residue threshold - we consider values higher than this threshold bad
settings.solver.conf.autoscale    =  0;      % [bool] post processing autoscaling
settings.solver.conf.goodlim      =  0.5;    % [1] all balls with a confidence higher than this threshold are accepted
settings.solver.conf.maybelim     =  0.2;    % [1] if no good balls are seen, then this limit is used
settings.solver.conf.nummaybe     =  1;      % [1] amount of 'maybe' balls to accept
% TODO tuning strategy
% TODO smoothener?
% solver settings - blacklist - NOT YET IMPLEMENTED
settings.solver.blist.floatz      =  0.50;   % [m] threshold above which a ball may be reported as floating
settings.solver.blist.floattime   =  2.0;    % [s] time after which a ball is assumed to be floating
% solver settings - analysis specific (not needed in robot implementation)
settings.solver.ana.frequency     = 10;      % [Hz] solver frequency
settings.solver.ana.perrobot      =  0;      % [bool] solve per robot individually
settings.solver.ana.filterage     =  1.0;    % [s] how far to look back in filter
settings.solver.ana.filtertype    = 'mamsd'; % [string] which filter to use (see fbt_solve_filter('list'))

% measurement filtering (GUI)
settings.measfilter.tmin          = 0;
settings.measfilter.tmax          = 1e20;
settings.measfilter.tcurr         = 0;
settings.measfilter.robots        = [1 1 1 1 1 1]; % all enabled

% visualization settings (GUI)
%NI settings.visualizer.fig1.connect  = 0;       % connect measurement with origin via dashed line
%NI settings.visualizer.fig1.speedfactor = 0.3;  % speed vector scaling
settings.visualizer.fig2.bx        = 1;      % plot ball x measurements against time in 2nd figure
settings.visualizer.fig2.by        = 1;      % same, for y
settings.visualizer.fig2.bz        = 0;      % same, for z (less interesting)
settings.visualizer.fig2.solutions = 1;      % plot solutions in 2nd figure
settings.visualizer.fig2.residuals = 0;      % plot solver residuals
settings.visualizer.fig2.position  = 1;      % plot position component (with velocity as quiver)
settings.visualizer.fig2.velocity  = 0;      % plot only velocity component
settings.visualizer.fig2.filter    = 0;      % plot filtered solutions instead of unfiltered
settings.visualizer.fig2.simulated = 1;      % plot simulated scene, if present
settings.visualizer.fig2.errors    = 1;      % plot errors w.r.t. simulated scene, if present
settings.visualizer.fig3.az        = 1;      % plot relative spherical coordinates against time in 2nd figure
settings.visualizer.fig3.el        = 1;      % similar
settings.visualizer.fig3.r         = 1;      % similar
settings.visualizer.fig3.conf      = 0;      % measurement confidence [0, 1]
settings.visualizer.fig3.cx        = 0;      % plot camera x measurements against time in 2nd figure
settings.visualizer.fig3.cy        = 0;      % same, for y
settings.visualizer.fig3.cphi      = 0;      % same, for phi

% simulator settings
settings.sim.tmax             =  0.15;       % [s] time frame during which we track the ball
settings.sim.freq             = 30.0;        % [Hz] camera sampling frequency
settings.sim.gravity          = v * -9.81;   % [m/s2] Z accelleration due to gravity
settings.sim.ball.drag        = v * 0.04;    % [1] drag coefficient, applied per simulation timestep
settings.sim.ball.bouncedp    = v * 0.2;     % [1] bounce energy dissipation coefficient

