function fbt_demo_20160609


% load single-robot capture data
%datadir      = fbt_datadir;
filename     = 'capture_r4_depth_20160809.txt';
measurements = fbt_meas_filter(fbt_meas_read(filename));
% cut away bad data during initialization
measurements = fbt_meas_filter(measurements, '-tmin', 82237680);
% select which phase to show
if 1
    measurements = fbt_meas_filter(measurements, '-tmax', 82237720);
else
    measurements = fbt_meas_filter(measurements, '-tmin', 82237726);
end
% begin without solver (slow)
settings     = fbt_settings;
%settings.visualizer.fig2.solutions = 0;

% start the GUI
close all; 
fbt_gui(measurements, settings);

