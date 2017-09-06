function fbt_test_performance_penalty


% Monte-Carlo simulation test case: penalty, only keeper sees the ball 
% (other teammates should be around y=0, looking at the back of opponent who is taking the penalty)
% ball is stationary and then suddenly starts moving with 10m/s, always into the corner at +x
% suppose the keeper only has 0.1sec to choose the right corner (see that vx > 0)
N            = 3;
scene        = fbt_scene_5;
settings     = fbt_settings;
settings.solver.tracker.xytol = 5;  % match settings
settings.solver.ana.frequency = 30; % robot heartbeat, default 10Hz is not enough
settings.sim.gravity = 0; % disable physics
settings.sim.ball.drag = 0; % disable physics
% if isoctave
%     expected = [0.20 0.50 0.75 1.00]; % this should improve upon implementing fbt_solve_bounce
% else
%     expected = [0.30 0.55 0.90 1.00]; % this should improve upon implementing fbt_solve_bounce
% end
responsetime = 0.05:0.03333:0.15;
for withnoise = 0:1
    if withnoise
        errors    = fbt_errors(1); % standard errors
        errors.az = 3 * errors.az; % increase azimuth noise a bit, to account for ballhandlers / blocked light
    else
        errors    = fbt_errors(0);
    end
    for withbounce = 0:1
        settings.solver.bounce.dt = 0.5 - 0.4 * withbounce; % enable/disable bounce detection
        for it = 1:numel(responsetime)
            got(it)  = sub_penalty(scene, settings, errors, responsetime(it), N);
        end
    end
end
%assertVectorsAlmostEqual(expected, got, 'absolute', 1e-1);



% end of main, sub functions below
    
function [scene, settings, errors] = sub_cbpre(scene, settings, errors, iseed)
    scene.ball.vx = (mod(iseed, 3) - 1) * scene.ball.vx;
    
function successrate = sub_penalty(scene, settings, errors, responsetime, N)
    settings.sim.tmax = responsetime + scene.ball.tstart; 
    result       = fbt_sim_monte_carlo(scene, settings, errors, '-N', N, '-cbpre', @sub_cbpre);
    % bad results are balls moving to other corner or even away from keeper
    % TODO
    result.final
    badseed      = find((result.final.vx < 0) | (result.final.vy > 0));
    successrate  = 1.0 - numel(badseed) / N;
    disp(sprintf('penalty direction success rate (responsetime=%.2fs): %d%%', responsetime, round(100 * successrate)));
    % debugging, plot first bad seed: 
    %  close all ; fbt_gui(fbt_sim_scene(scene, settings, errors, badseed(1)), settings);


