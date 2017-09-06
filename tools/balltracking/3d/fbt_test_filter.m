function fbt_test_filter

t0           = tic;
scene        = fbt_scene_4;
errors       = fbt_errors; % nominal
settings     = fbt_settings;
settings.sim.tmax = 300; % make simulation run quite long, so we can optimize performance
measurements = fbt_sim_scene(scene, settings, errors, 37);
N            = measurements.n;
elapsed      = toc(t0);
assert(elapsed < 15);
t0           = tic;
meas_filt    = fbt_meas_filter(measurements, '-xmin', -1, '-xmax', 1); % no effect
elapsed      = toc(t0);
assert(elapsed < 8);
assert(meas_filt.n == N);
t0           = tic;
meas_filt    = fbt_meas_filter(measurements, '-tmin', 0, '-tmax', 1);
elapsed      = toc(t0);
assert(elapsed < 8);
assert(meas_filt.n == 93);
1;
%assert(strcmp(fbt_meas_2str(measurements), fbt_meas_2str(fbt_meas_read('testdata04.txt'))));
%close all ; fbt_meas_plot(measurements);
%fbt_meas_write(measurements, 'testdata04.txt');

