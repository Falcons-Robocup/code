function fbt_test_performance_bounce

% test case: evaluate bounce detection by simulating a vertically bouncing ball, 
% three spectators, ideally positioned
% evaluate observed error w.r.t. simulated path so we can tune parameters and play with bounce detection
N            = 10;
scene        = fbt_scene_6;
errors       = fbt_errors(1); % standard errors
errors.ts    = 0; % but disable time randomizers because it messes up our index filter
errors.tr    = 0; 
settings     = fbt_settings;
settings.solver.tracker.xytol = 5;  % match settings
settings.solver.ana.frequency = 30; % robot heartbeat, default 10Hz is not enough
settings.sim.tmax = 3.0;
idxmask      = [40:51 54:64]; % filter out two time indices immediately after bounce, we cannot detect those
result       = fbt_sim_monte_carlo(scene, settings, errors, N, idxmask);
got_posx = sprintf('   x position error: (%6.2f, %6.2f, %6.2f, %6.2f)', sub_stats(result.kpi.errpos.x));
got_posy = sprintf('   y position error: (%6.2f, %6.2f, %6.2f, %6.2f)', sub_stats(result.kpi.errpos.y));
got_posz = sprintf('   z position error: (%6.2f, %6.2f, %6.2f, %6.2f)', sub_stats(result.kpi.errpos.z));
got_velx = sprintf('   x velocity error: (%6.2f, %6.2f, %6.2f, %6.2f)', sub_stats(result.kpi.errvel.x));
got_vely = sprintf('   y velocity error: (%6.2f, %6.2f, %6.2f, %6.2f)', sub_stats(result.kpi.errvel.y));
got_velz = sprintf('   z velocity error: (%6.2f, %6.2f, %6.2f, %6.2f)', sub_stats(result.kpi.errvel.z));
disp(sprintf('bounce detection performance (min, max, avg, std):'));
disp(got_posx); disp(got_posy); disp(got_posz); 
disp(got_velx); disp(got_vely); disp(got_velz); 
expected_posx = '   x position error: (  0.01,   0.02,   0.02,   0.00)';
expected_posy = '   y position error: (  0.01,   0.03,   0.02,   0.01)';
expected_posz = '   z position error: (  0.08,   0.10,   0.09,   0.01)';
expected_velx = '   x velocity error: (  0.08,   0.30,   0.18,   0.06)';
expected_vely = '   y velocity error: (  0.07,   0.48,   0.24,   0.12)';
expected_velz = '   z velocity error: (  2.21,   2.28,   2.25,   0.02)';
assert(strcmp(expected_posx, got_posx));   
assert(strcmp(expected_posy, got_posy));   
assert(strcmp(expected_posz, got_posz));   
assert(strcmp(expected_velx, got_velx));   
assert(strcmp(expected_vely, got_vely));   
assert(strcmp(expected_velz, got_velz));   


% end of main, sub functions below
    
function stats = sub_stats(values)
    stats = [min(values) max(values) mean(values) std(values)];
    