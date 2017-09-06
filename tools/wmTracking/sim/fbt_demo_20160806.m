function fbt_demo_20160606


% load triple-robot triple-ball capture data
%datadir      = fbt_datadir;
filename     = 'capture_20160806_scene1.txt';
measurements = fbt_meas_filter(fbt_meas_read(filename));
% bugfix azimuth a bit (coordinate transformation was wrong, fixed 9aug)
idx_az       = fbt_meas_idx('az');
measurements.data(:, idx_az) = -measurements.data(:, idx_az);
idx_bx       = fbt_meas_idx('bx'); % clear cached ball FCS positions, recalculate
measurements.data(:, idx_bx:end) = [];

% % cut away bad data during initialization
% measurements = fbt_meas_filter(measurements, '-tmin', 82237680);
% % select which phase to show
% if 1
%     measurements = fbt_meas_filter(measurements, '-tmax', 82237720);
% else
%     measurements = fbt_meas_filter(measurements, '-tmin', 82237726);
% end
measurements = fbt_meas_filter(measurements, '-xmin', -1.2, '-ymax', -4.9);

% begin without solver (slow)
settings     = fbt_settings;
settings.solver.bounce.dt = 0.5; % disable bounce detection for this crappy scene
%settings.visualizer.fig2.solutions = 0;

%error('TODO optimize');
% start the GUI
close all; 
fbt_gui(measurements, settings);

