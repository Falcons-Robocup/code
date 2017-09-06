function measurements = fbt_sim_robot(scene, settings, errors, irobot, ball_positions, varargin)
% simulate how a robot perceives the ball


% settings
CAM_Z         = 0.38;
%CAM_R         = 0.15; % TODO

% calculate time sampling
dt            = 1.0 / settings.sim.freq;
t             = [0:dt:settings.sim.tmax]';
n             = numel(t);

% add noise specific for this robot
t             = t + rand * errors.tr; % systematic CPU time offset
t             = t + (rand(n,1) - 0.0) * errors.ts; % random error on each sample; prevent negative time

% calculate robot position for each timestamp
robot_pos     = scene.robot(irobot);
xr            = robot_pos.x + t * robot_pos.vx;
yr            = robot_pos.y + t * robot_pos.vy;
phir          = robot_pos.phi + t * robot_pos.vphi;

% add noise to robot position
xr            = xr + errors.loc * (2 * rand(n,1) - 1);
yr            = yr + errors.loc * (2 * rand(n,1) - 1);
phir          = phir + 0.3 * errors.loc * (2 * rand(n,1) - 1);

% get ball positions (undistorted)
bx            = ball_positions.x;
by            = ball_positions.y;
bz            = ball_positions.z;

% convert to RCS
[dx, dy, dz]  = fct_fcs2rcs(xr, yr, phir, bx, by, bz);
% correct camera mounting height
dz            = bz - CAM_Z;
xc            = xr; % TODO: use camera mounting offset CAM_R
yc            = yr; % TODO: use camera mounting offset CAM_R
zc            = repmat(CAM_Z, [n 1]); 
% TODO: proper solution would be to intrude CCS (Camera Coordinate System) which is basically RCS with an offset (camera mounting position w.r.t. robot)

% convert to spherical coordinates
[az, el, r]   = cart2sph(dx, dy, dz);
az            = az - pi/2;

% add RELATIVE noise to ball measurements
az            = az .* (1.0 + errors.az * (2*rand(n,1) - 1.0));
el            = el .* (1.0 + errors.el * (2*rand(n,1) - 1.0));
r             = r  .* (1.0 + errors.r  * (2*rand(n,1) - 1.0));

% store for use in solver
id            = repmat(irobot, [n 1]);
conf          = repmat(1, [n 1]); % TODO 
cam           = repmat(0, [n 1]); % TODO 
measurements.data = [id cam t xc yc zc phir az el r conf];
measurements.n    = n;
