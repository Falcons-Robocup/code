function data = fbt_test_vdl_20160929()
% stimulate data from testmatch VDL, used in testInput03.txt


% load measurements
% * from t=15s to t=30s
%   * otherwise it would take ages... 
%   * and it would not fit in our testdata .txt file (260KB hard repo file size limit)
filename          = 'zut/20160929_testmatch_VDL_2ndhalf_frontBalls.txt';
measurements      = fbt_meas_read(filename, '-verbose', '-tmax', 86642370, '-tmin', 86642355);

% workaround: all measurements were done with frontcam but marked as omni
measurements.data(:, fbt_meas_idx('cam')) = 0;

% run solver
settings          = fbt_settings;
settings.solver.tracker.numwarn = 3;
settings.solver.bounce.dt       = 10; % disable bounce detection
settings.solver.tracker.xytol   = 2.5;
settings.solver.conf.omnipref   = 0;
settings.solver.core.speed      = 0; % for now no speed
settings.solver.verbosity       = 5;
solutions         = fbt_solve_sequence(settings, measurements);

% store output
data.measurements = measurements;
data.solutions    = solutions;

