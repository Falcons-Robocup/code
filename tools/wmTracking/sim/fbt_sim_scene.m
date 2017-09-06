function [measurements, ball_positions] = fbt_sim_scene(scene, settings, errors, randseed)
% simulate measurements for a scene:
%  * simulate ball movement
%  * TODO simulate robot movement
%  * simulate how each robot perceives the ball


% make simulations reproducable
if nargin > 3
    rand('seed', randseed); % NOTE: implementation differs between Matlab & Octave!
end

% calculate time sampling
dt               = 1.0 / settings.sim.freq;
t                = [0:dt:settings.sim.tmax]';
n                = numel(t);

% generate ball positions in FCS
ball_positions   = fbt_sim_ball(scene.ball, t, settings);

% process each robot
measurements     = [];
for irobot = 1:scene.num_robots
    m_thisrobot  = fbt_sim_robot(scene, settings, errors, irobot, ball_positions);
    measurements = fbt_meas_merge(measurements, m_thisrobot);
end

