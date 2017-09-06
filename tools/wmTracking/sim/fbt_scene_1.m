function scene = fbt_scene_1
% JFEI 20160706 worldModelV2 Falcons Ball Tracking rapid prototyping
% create the scene from Andre (email 20160706 16:06)


% robot and ball positions in FCS
scene.num_robots = 2;
scene.robot(1).x    =   0.0; % normal goalie position
scene.robot(1).y    =  -9.0;
scene.robot(1).phi  =   1.57;
scene.robot(1).z    =   0.7; % mounting height of frontcam
scene.robot(1).vx   =   0.0;
scene.robot(1).vy   =   0.0;
scene.robot(1).vphi =   0.0;
scene.robot(2).x    =   2.0; % observer behind the ball with a slight viewing angle
scene.robot(2).y    =   0.0;
scene.robot(2).phi  =   4.71;
scene.robot(2).z    =   0.7;
scene.robot(2).vx   =   0.0;
scene.robot(2).vy   =   0.0;
scene.robot(2).vphi =   0.0;
scene.ball.x        =   0.0; % ball from penalty spot into left-top corner
scene.ball.y        =  -6.0; % moving with about 10m/s
scene.ball.z        =   0.0;
scene.ball.vx       =   2.2; 
scene.ball.vy       = -10.0;
scene.ball.vz       =   2.2;

% settings
scene.settings.errors.az   =  0.05;  % relative error 3sigma
scene.settings.errors.el   =  0.05;  % relative error 3sigma
scene.settings.errors.r    =  0.35;  % relative error 3sigma
scene.settings.errors.loc  =  0.05;  % [m] 3sigma
scene.settings.errors.ts   =  0.005; % [s] 3sigma random delay per sample
scene.settings.errors.tr   =  0.015; % [s] 3sigma random systematic delay per robot (to be solved with NTP)
scene.settings.time.tmax   =  0.15;   % [s] time frame during which we track the ball
scene.settings.time.freq   = 30.0;   % [Hz] camera sampling frequency
