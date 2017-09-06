function fbt_test_scenes

close all; 

% scene 1
scene        = fbt_scene_1;
errors       = fbt_errors(1);
settings     = fbt_settings;
settings.solver.tracker.xytol = 10; % let go of the limit
settings.solver.ana.perrobot = 1; % let go of the limit
data         = fbt_sim_solve(scene, settings, errors, 37);
fbt_plot(data);
% debugging: fbt_gui(data.measurements, settings, scene);

% scene 2
scene        = fbt_scene_2;
errors       = fbt_errors(1);
settings     = fbt_settings;
settings.solver.tracker.xytol = 10; % let go of the limit
settings.sim.tmax = 1.3;
data         = fbt_sim_solve(scene, settings, errors, 37);
fbt_plot(data);

% scene 3
scene        = fbt_scene_3;
errors       = fbt_errors(1);
settings     = fbt_settings;
settings.solver.tracker.xytol = 10; % let go of the limit
data         = fbt_sim_solve(scene, settings, errors, 37);
fbt_plot(data);


