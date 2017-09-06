function data = fbt_sim_solve(scene, settings, errors, randseed)
% simulate a scene and call solver


% fill in missing arguments
if nargin < 3
    errors        = fbt_errors;
end
if nargin < 2
    settings      = fbt_settings;
end

% simulate measurements in each robot RCS
[measurements, ballsim] = fbt_sim_scene(scene, settings, errors, randseed);

% solve for each robot, to stress the importance of robot positioning
if (scene.num_robots > 1) && settings.solver.ana.perrobot
    for irobot = 1:scene.num_robots
        solution.robot{irobot} = sub_solve(scene, settings, fbt_meas_filter(measurements, '-robot', irobot));
    end
end
solution.merged   = sub_solve(scene, settings, measurements);

% store output
data.scene        = scene;
data.scene.ballsim = ballsim; % nesting hack for fbt_gui ...
data.measurements = measurements;
data.solution     = solution;


function result = sub_solve(scene, settings, varargin)
    % solve
    result          = fbt_solve_sequence(settings, varargin{:});
    % compare result with known solution - moved to fbt_kpi
    
