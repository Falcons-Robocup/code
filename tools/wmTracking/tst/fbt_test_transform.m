function fbt_test_transform

[x, y, z]       = fct_fcs2rcs(0, 0, 0, 0, 0, 0);
assertVectorsAlmostEqual([x y z], [0, 0, 0]);

[x, y, z]       = fct_fcs2rcs(0, 0, 0, 1, 1, 1);
assertVectorsAlmostEqual([x y z], [-1 1 1]);

[x, y, z]       = fct_fcs2rcs(-4, 0, 0, 1, 1, 1);
assertVectorsAlmostEqual([x y z], [-1 5 1]);

[x, y, z]       = fct_fcs2rcs(-4, 0, pi, 1, 1, 1);
assertVectorsAlmostEqual([x y z], [1 -5 1]);

[x, y, z]       = fct_fcs2rcs(-4, -6, pi/2, -2, 1, 1);
assertVectorsAlmostEqual([x y z], [2 7 1]);

% vectorize
[x, y, z]       = fct_fcs2rcs([-4; 0], [-6; 0], [pi/2; 0], [-2; 1], [1; 1], [1; 1]);
assertVectorsAlmostEqual([x y z], [2 7 1; -1 1 1]);


% identity check
N               = 500;
robot_pos.x     = rand(N,1);
robot_pos.y     = rand(N,1);
robot_pos.phi   = rand(N,1);
robot_pos.z     = rand(N,1);
bx              = rand(N,1);
by              = rand(N,1);
bz              = rand(N,1);
[x, y, z]       = fct_fcs2rcs(robot_pos.x, robot_pos.y, robot_pos.phi, bx, by, bz);
[xx, yy, zz]    = fct_rcs2fcs(robot_pos.x, robot_pos.y, robot_pos.phi, x, y, z);
assertVectorsAlmostEqual([bx, by, bz], [xx, yy, zz]);


