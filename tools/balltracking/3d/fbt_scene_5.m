function scene = fbt_scene_5
% performance test scene (see explanation in fbt_test_performance)

% robot and ball positions in FCS
scene.num_robots    = 1;
scene.robot(1).x    =   0.0; % normal goalie position
scene.robot(1).y    =  -9.0;
scene.robot(1).phi  =   1.57;
scene.robot(1).z    =   0.7; % mounting height of frontcam
scene.robot(1).vx   =   0.0;
scene.robot(1).vy   =   0.0;
scene.robot(1).vphi =   0.0;
scene.ball.x        =   0.0; % ball from penalty spot into left-top corner
scene.ball.y        =  -6.0; % moving with about 10m/s
scene.ball.z        =   0.0;
scene.ball.tstart   =   2.0;
scene.ball.vx       =   2.2; 
scene.ball.vy       = -10.0;
scene.ball.vz       =   2.2;
