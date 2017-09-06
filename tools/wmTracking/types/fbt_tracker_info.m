function fbt_tracker_info(tracker, varargin)
% show some info on tracker



t = tracker.measurements.data(:, fbt_meas_idx('t'));
disp(sprintf('details for tracker #%d:', tracker.id));
disp(sprintf('  #measurements = %d', tracker.measurements.n));
disp(sprintf('           told = %s', fbt_time_float2str(tracker.t0)));
disp(sprintf('           tmin = %s', fbt_time_float2str(min(t))));
disp(sprintf('           tmax = %s', fbt_time_float2str(max(t))));
disp(sprintf('       solution = [%6.2f %6.2f %6.2f]', tracker.solution.x, tracker.solution.y, tracker.solution.z));
disp(sprintf('           conf = %.3f', tracker.conf));
