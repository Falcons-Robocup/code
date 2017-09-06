function fbt_test_sim

scene         = fbt_scene_2;
scene.ball.vy = -10;
errors        = fbt_errors; % nominal
settings      = fbt_settings(0);
measurements  = fbt_sim_scene(scene, settings, errors, 37);
assert(strcmp(fbt_meas_2str(measurements), fbt_meas_2str(fbt_meas_read('testdata04.txt'))));
%close all ; fbt_meas_plot(measurements);
%fbt_meas_write(measurements, 'testdata04.txt');

