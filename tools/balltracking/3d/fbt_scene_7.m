function scene = fbt_scene_7
% test scene


% robot and ball positions in FCS
scene.num_robots = 1;
scene.robot(1).x    =   0.0; % normal goalie position
scene.robot(1).y    =  -9.0;
scene.robot(1).phi  =   1.57;
scene.robot(1).z    =   0.7; % mounting height of frontcam
scene.robot(1).vx   =   0.0;
scene.robot(1).vy   =   0.0;
scene.robot(1).vphi =   0.0;
scene.ball.x        =   0.0; % ball from center spot straight towards goalie
scene.ball.y        =   0.0;
scene.ball.z        =   0.0;
scene.ball.vx       =   0.0; 
scene.ball.vy       =  -8.0;
scene.ball.vz       =   0.0;
