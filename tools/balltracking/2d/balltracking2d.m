function balltracking2d

close all;

% robot position: (x,y,phi)
robot1 = [-3.0 -1.0 -0.1];
robot2 = [ 2.0  0.0  4.0];

% ball measurements: (alpha, r)
meas1  = [-0.1 4.0];
meas2  = [ 0.1 2.0];

% convert ball measurement from polar to cartesian (RCS), then to FCS, for plotting
ball1  = translate(robot1(1), robot1(2)) * rotate(-robot1(3)) * pad1(pol2cart(meas1));
ball2  = translate(robot2(1), robot2(2)) * rotate(-robot2(3)) * pad1(pol2cart(meas2));

% plot the scene
figure; hold on;
plot(robot1(1), robot1(2), 'r+');
plot(robot2(1), robot2(2), 'b+');
plot(ball1(1), ball1(2), 'ro');
plot(ball2(1), ball2(2), 'bo');
axis([-4 3 -3 1]);
plot([robot1(1) ball1(1)], [robot1(2) ball1(2)], 'r:');
plot([robot2(1) ball2(1)], [robot2(2) ball2(2)], 'b:');
axis equal;


% construct a system to solve
% here we don't have our falcons-specific pi/2 FCS/RCS offset yet, so x in "RCS" is the distance to ball
A1 = rotate(robot1(3)) * translate(-robot1(1), -robot1(2)); % FCS to "RCS"
A2 = rotate(robot2(3)) * translate(-robot2(1), -robot2(2));
rhs1 = pad1(pol2cart(meas1));
rhs2 = pad1(pol2cart(meas2));
A = [A1; A2]
rhs = [rhs1; rhs2]
w = 1; % set the distance weight factor here
W = diag([w 1 1 w 1 1]); % reduce weighting on x (TODO: falcons RCS should reduce weight on y, extra quarter rotation)
p = (W*A) \ (W*rhs)


% plot the solution
plot(p(1), p(2), 'k*');
% the lower w is chosen, the closer the solution comes to ray intersection

