function scene = fbt_scene_6
% performance test scene (see explanation in fbt_test_performance)

% robot and ball positions in FCS
scene.num_robots    = 3;
scene.robot(1).x    =   0.0; % three robots in a triangle, exactly facing the ball
scene.robot(1).y    =   4.0;
scene.robot(1).phi  =   1.5 * pi;
scene.robot(1).z    =   0.7; % mounting height of frontcam
scene.robot(1).vx   =   0.0;
scene.robot(1).vy   =   0.0;
scene.robot(1).vphi =   0.0;
scene.robot(2).x    =  -4.0;
scene.robot(2).y    =  -2.0;
scene.robot(2).phi  =   pi / 6.0;
scene.robot(2).z    =   0.7; % mounting height of frontcam
scene.robot(2).vx   =   0.0;
scene.robot(2).vy   =   0.0;
scene.robot(2).vphi =   0.0;
scene.robot(3).x    =   4.0;
scene.robot(3).y    =  -2.0;
scene.robot(3).phi  =   pi * 5.0 / 6.0;
scene.robot(3).z    =   0.7; % mounting height of frontcam
scene.robot(3).vx   =   0.0;
scene.robot(3).vy   =   0.0;
scene.robot(3).vphi =   0.0;
scene.ball.x        =   0.0; % ball at center of field, freely falling from a certain height
scene.ball.y        =   0.0;
scene.ball.z        =   5.0;
scene.ball.tstart   =   1.0;
scene.ball.vx       =   0.0;
scene.ball.vy       =   0.0;
scene.ball.vz       =   0.0;
