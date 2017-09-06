function fbt_plot(varargin)
% plot scene, measurements, or solution(s)



% settings
robot_colors = fbt_colors;
merged_color = 'k';
ball_color   = 'b';

% TODO parse options?
args = varargin;

% no need to initialize figure if we start with calling fbt_meas_plot
figure; hold on;

% plot whatever is given
plotted_meas = false;
for iarg = 1:numel(args)
    arg = args{iarg};
    assert(isstruct(arg));
    if isfield(arg, 'data')
        fbt_meas_plot(arg);
        plotted_meas = true;
    elseif isfield(arg, 'vx')
        assert(plotted_meas); % plotting a solution only makes sense if measurements have been plotted
        fbt_sol_plot(arg);
    elseif isfield(arg, 'measurements')
        % sim struct: first the measurements
        fbt_meas_plot(getfield(arg, 'measurements'));
        % now solutions per robot
        solution = getfield(arg, 'solution');
        if isfield(solution, 'robot')
            for irobot = 1:numel(solution.robot)
                fbt_sol_plot(solution.robot{irobot}, '-color', robot_colors(irobot));
            end
        end
        fbt_sol_plot(solution.merged, '-color', 'k');
    end
end
