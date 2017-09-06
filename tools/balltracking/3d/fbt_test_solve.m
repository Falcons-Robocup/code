function fbt_test_solve

%% test the core function for direct fit: fbt_solve_fit

% ball at (0,0,0), three robots with each a single measurement, fit without speed
settings     = fbt_settings;
measurements = fbt_meas_read('testdata01.txt');
settings.solver.core.speed = 0;
solution     = fbt_solve_fit(settings, measurements);
expected     = [0 0 0.38 0 0 0]; % measurements all planar, but design matrix takes camera mounting height into account
got          = [solution.x solution.y solution.z solution.vx solution.vy solution.vz];
assertVectorsAlmostEqual(expected, got, 'absolute', 1e-4);

% same, but now with slightly reduced radius, to check averaging
% first, if we use a nonzero weight, the solution will deviate a bit from the exact solution
idxr         = fbt_meas_idx('r');
measurements.data(:,idxr) = 0.9 * measurements.data(:,idxr);
settings.solver.core.weight = 0.2;
solution     = fbt_solve_fit(settings, measurements);
expected     = [-0.0017 -0.0079 0.359 0 0 0];
got          = [solution.x solution.y solution.z solution.vx solution.vy solution.vz];
assertVectorsAlmostEqual(expected, got, 'absolute', 1e-4);

% now we let go of the weight, effectively intersecting the lines
settings.solver.core.weight = 0.0;
solution     = fbt_solve_fit(settings, measurements);
expected     = [0 0 0.38 0 0 0];
got          = [solution.x solution.y solution.z solution.vx solution.vy solution.vz];
assertVectorsAlmostEqual(expected, got, 'absolute', 1e-4);

% ball at (1,1,1), three robots with each a single measurement, fit without speed
settings     = fbt_settings;
measurements = fbt_meas_read('testdata02.txt');
settings.solver.core.speed = 0;
solution     = fbt_solve_fit(settings, measurements);
expected     = [1 1 1.38 0 0 0];
got          = [solution.x solution.y solution.z solution.vx solution.vy solution.vz];
assertVectorsAlmostEqual(expected, got, 'absolute', 1e-4);

% ball moving towards goalie, no noise
settings     = fbt_settings;
measurements = fbt_meas_read('testdata03.txt');
solution     = fbt_solve_fit(settings, measurements);
expected     = [0.60149 -3.00496 0.98453 2.00507 -10.01654 -1.31837];
got          = [solution.x solution.y solution.z solution.vx solution.vy solution.vz];
%close all ; fbt_meas_plot(measurements); fbt_sol_plot(solution);
assertVectorsAlmostEqual(expected, got, 'absolute', 1e-4);



%% test the top-level discriminator (tracker) and KPI's

% simulate scene 2 to verify ball speed tracking
% see also: fbt_demo_sim
scene        = fbt_scene_2;
errors       = fbt_errors(1);
settings     = fbt_settings;
settings.solver.tracker.xytol = 10; % let go of the limit -- otherwise a few outliers will occur
settings.solver.core.weight   = 0.1;
settings.solver.conf.goodlim  = -1; % disable confidence heuristic -- all balls are good in this simulation
settings.sim.gravity = 0; % disable physics
settings.sim.ball.drag = 0; % disable physics
settings.sim.tmax = 1.3;
data         = fbt_sim_solve(scene, settings, errors, 37);
% debugging: close all ; fbt_gui(data.measurements, settings, data.scene);
% check solution stability
kpi          = fbt_kpi(settings, data.measurements, data.solution.merged, '-scene', data.scene);
assert(kpi.numballs == 1);
assertVectorsAlmostEqual(kpi.filtered.min.x,     -0.02, 'absolute', 2e-2);
assertVectorsAlmostEqual(kpi.filtered.max.x,      0.02, 'absolute', 2e-2);
assertVectorsAlmostEqual(kpi.filtered.min.vy,    -5.2,  'absolute', 2e-1);
assertVectorsAlmostEqual(kpi.filtered.max.vy,    -4.7,  'absolute', 3e-1);
assertVectorsAlmostEqual(kpi.filtered.min.z,      0.00, 'absolute', 1e-2);
assertVectorsAlmostEqual(kpi.filtered.max.z,      0.01, 'absolute', 1e-2);
assertVectorsAlmostEqual(kpi.stab.pos,            1.1,  'absolute', 2e-1);
assertVectorsAlmostEqual(kpi.stab.vel,            0.1,  'absolute', 1e-1);
if ~isoctave
    assertVectorsAlmostEqual(max(abs(kpi.errors.x)),  0.03, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.y)),  0.07, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.z)),  0.01, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.vx)), 0.06, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.vy)), 0.03, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.vz)), 0.01, 'absolute', 1e-2);
end

% simulate scene 4 to verify ball position stability
% see also: fbt_demo_sim
scene        = fbt_scene_4;
errors       = fbt_errors(1);
settings     = fbt_settings;
settings.solver.core.weight = 0.1; % quite sensitive, results with weight=0.2 are much worse
settings.solver.conf.goodlim  = -1; % disable confidence heuristic -- all balls are good in this simulation
settings.sim.gravity = 0; % disable physics
settings.sim.ball.drag = 0; % disable physics
settings.sim.tmax = 2.0;
rand('seed', 37);
data         = fbt_sim_solve(scene, settings, errors, 37);
% debugging: close all ; fbt_gui(data.measurements, settings, data.scene);
% check solution stability
kpi          = fbt_kpi(settings, data.measurements, data.solution.merged, '-scene', data.scene);
assert(kpi.numballs == 1);
assertVectorsAlmostEqual(kpi.filtered.min.x,      0.0,  'absolute', 1e-1);
assertVectorsAlmostEqual(kpi.filtered.min.y,     -4.0,  'absolute', 1e-1);
assertVectorsAlmostEqual(kpi.filtered.min.z,      0.0,  'absolute', 2e-1);
assertVectorsAlmostEqual(kpi.filtered.max.x,      0.0,  'absolute', 1e-1);
assertVectorsAlmostEqual(kpi.filtered.max.y,     -4.0,  'absolute', 1e-1);
assertVectorsAlmostEqual(kpi.filtered.max.z,      0.0,  'absolute', 2e-1);
assertVectorsAlmostEqual(kpi.stab.pos,            0.02, 'absolute', 1e-2);
assertVectorsAlmostEqual(kpi.stab.vel,            0.01, 'absolute', 1e-2);
if ~isoctave
    assertVectorsAlmostEqual(max(abs(kpi.errors.x)),  0.01, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.y)),  0.04, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.z)),  0.01, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.vx)), 0.02, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.vy)), 0.04, 'absolute', 1e-2);
    assertVectorsAlmostEqual(max(abs(kpi.errors.vz)), 0.01, 'absolute', 1e-2);
end

% simulate scene 6 to test bounce detection
% JFEI 20161028 bounce detection is postponed to phase2
% N            = 1;
% scene        = fbt_scene_6;
% errors       = fbt_errors(0); % NO errors -- see also fbt_test_performance 
% settings     = fbt_settings;
% settings.solver.tracker.xytol = 5;  % match settings
% settings.solver.conf.goodlim  = -1; % disable confidence heuristic -- all balls are good in this simulation
% settings.solver.ana.frequency = 30; % robot heartbeat, default 10Hz is not enough
% settings.solver.ana.perrobot = 0;
% settings.sim.tmax = 3.0;
% iseed        = 1;
% data         = fbt_sim_solve(scene, settings, errors, iseed);
% kpi          = fbt_kpi(settings, data.measurements, data.solution.merged, '-scene', data.scene);
% %close all ; fbt_gui(data.measurements, settings, data.scene);
% idxmask      = 51:55; % right around the bounce (TODO: poor man.. sensitive)
% assertVectorsAlmostEqual(kpi.errors.z(idxmask),  [ 0.0;  0.0;  0.2;  0.0;  0.0], 'absolute', 1e-1);
% assertVectorsAlmostEqual(kpi.errors.vz(idxmask), [-0.5; -0.5; 10.7; -0.7; -0.7], 'absolute', 2e-1);
% assertVectorsAlmostEqual(kpi.errors.x(idxmask),  [ 0.0;  0.0;  0.0;  0.0;  0.0], 'absolute', 1e-2);
% assertVectorsAlmostEqual(kpi.errors.vx(idxmask), [ 0.0;  0.0;  0.0;  0.0;  0.0], 'absolute', 1e-2);
% assertVectorsAlmostEqual(kpi.errors.y(idxmask),  [ 0.0;  0.0;  0.0;  0.0;  0.0], 'absolute', 1e-2);
% assertVectorsAlmostEqual(kpi.errors.vy(idxmask), [ 0.0;  0.0;  0.0;  0.0;  0.0], 'absolute', 1e-2);
