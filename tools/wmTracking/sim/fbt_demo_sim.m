function fbt_demo_sim


% simulate a scene
scene        = fbt_scene_2;
errors       = fbt_errors(1);
settings     = fbt_settings;
settings.solver.tracker.xytol = 10; % let go of the limit -- otherwise a few outliers will occur
settings.sim.tmax = 1.3;
settings.sim.gravity = 0; % disable physics
settings.sim.ball.drag = 0; % disable physics
rand('seed', 37);
measurements = fbt_sim_scene(scene, settings, errors);

% start the GUI
close all; 
fbt_gui(measurements, settings); % TODO pass scene

