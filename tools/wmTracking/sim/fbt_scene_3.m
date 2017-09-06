function scene = fbt_scene_3


% robot and ball positions in FCS
scene.num_robots    =   3;
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
scene.robot(3).x    =  -3.0; % observer at the left (from goalie view)
scene.robot(3).y    =  -3.0;
scene.robot(3).phi  =   0.0;
scene.robot(3).z    =   0.7;
scene.robot(3).vx   =   0.0;
scene.robot(3).vy   =   0.0;
scene.robot(3).vphi =   0.0;
scene.ball.x        =   0.0; % ball from penalty spot into left-top corner
scene.ball.y        =  -6.0; % moving with about 10m/s
scene.ball.z        =   0.0;
scene.ball.vx       =   2.2; 
scene.ball.vy       = -10.0;
scene.ball.vz       =   2.2;
