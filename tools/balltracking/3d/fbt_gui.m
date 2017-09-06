function fbt_gui(measurements, varargin)
% given is a set of measurements and possibly some settings
% solve according to settings, then provide controls to manipulate the solver realtime



% use default settings if not provided
settings = fbt_settings;
scene    = [];
for iarg = 1:numel(varargin)
    if isfield(varargin{iarg}, 'measfilter')
        settings = varargin{iarg};
    end
    % store scene for use in kpi calculation
    if isfield(varargin{iarg}, 'robot')
        scene = varargin{iarg};
    end
end

% initialize measurement filtering
id    = measurements.data(:, fbt_meas_idx('id'));
t     = measurements.data(:, fbt_meas_idx('t'));
settings.measfilter.tmin          = min(t);
settings.measfilter.tmax          = max(t);
settings.measfilter.tcurr         = max(t);
settings.measfilter.robots        = zeros(1, 6);
settings.measfilter.robots(unique(sort(id))) = 1;

% TODO load dialog?

% create a figure and axes (1: 3D scene of measurements; 2: measurements against time)
f = figure('Visible', 'off', 'Units', 'pixels', 'Position', [5 45 1350 600], 'Name', 'Falcons BallTracking GUI');
ax{1} = axes('Units', 'pixels', 'Position', [270  50 580 440], 'Parent', f, 'Tag', 'ax1');
ax{1}.Title.String = '3D scene of robots, measurements and solutions (trackers)';
ax{1}.XLabel.String = 'x';
ax{1}.YLabel.String = 'y';
ax{1}.ZLabel.String = 'z';
grid on;
ax{2} = axes('Units', 'pixels', 'Position', [920  50 400 200], 'Parent', f, 'Tag', 'ax2');
ax{2}.Title.String = '(x,y,z) ball input and solutions against time';
grid on;
ax{3} = axes('Units', 'pixels', 'Position', [920 300 400 200], 'Parent', f, 'Tag', 'ax3');
ax{3}.Title.String = 'raw input against time';
grid on;

% top level time sliders and labels
controls.tmin     = uicontrol(f, 'Position', [280 575 1000  20], 'Style', 'slider', 'String', 'tmin', 'Min', settings.measfilter.tmin, 'Max', settings.measfilter.tmax, 'Value', settings.measfilter.tmin, 'Callback', @sub_filter_measurements_cb); 
controls.tmin_lab = uicontrol(f, 'Position', [10  575  260  20], 'Style', 'text', 'String', 'tmin');
controls.tmax     = uicontrol(f, 'Position', [280 550 1000  20], 'Style', 'slider', 'String', 'tmax', 'Min', settings.measfilter.tmin, 'Max', settings.measfilter.tmax, 'Value', settings.measfilter.tmax, 'Callback', @sub_filter_measurements_cb); 
controls.tmax_lab = uicontrol(f, 'Position', [10  550  260  20], 'Style', 'text', 'String', 'tmax');
controls.tcur     = uicontrol(f, 'Position', [280 525 1000  20], 'Style', 'slider', 'String', 'tcurr', 'Min', settings.measfilter.tmin, 'Max', settings.measfilter.tmax, 'Value', settings.measfilter.tmax, 'Callback', @sub_filter_measurements_cb); 
controls.tcur_lab = uicontrol(f, 'Position', [10  525  260  20], 'Style', 'text', 'String', 'tcurr');

% solver controls + dump button
controls.dump   = uicontrol(f, 'Position', [10 440  70  30], 'String', 'dump data', 'Tag', 'button', 'Callback', @sub_dump_cb);
controls.speed  = uicontrol(f, 'Position', [90 440 100  30], 'String', 'disable speed', 'Tag', 'button', 'Callback', @sub_toggle_withspeed_cb);
controls.weight = uicontrol(f, 'Position', [70 410 120  20], 'Style', 'slider', 'String', 'weight', 'Min', 0, 'Max', 1, 'Value', settings.solver.core.weight, 'Callback', @sub_solver_settings_cb); 
controls.wlabel = uicontrol(f, 'Position', [10 405  60  30], 'Style', 'text', 'String', 'weight');
controls.freq   = uicontrol(f, 'Position', [70 380 120  20], 'Style', 'slider', 'String', 'freq', 'Min', 5, 'Max', 30, 'Value', settings.solver.ana.frequency, 'Callback', @sub_solver_settings_cb); 
controls.flabel = uicontrol(f, 'Position', [10 375  60  30], 'Style', 'text', 'String', 'freq');
controls.asolve = uicontrol(f, 'Position', [70 350 120  20], 'Style', 'slider', 'String', 'asol', 'Min', 0, 'Max', 1.0, 'Value', settings.solver.bounce.age, 'Callback', @sub_solver_settings_cb); 
controls.alabel = uicontrol(f, 'Position', [10 345  60  30], 'Style', 'text', 'String', 'asol');
controls.tout   = uicontrol(f, 'Position', [70 320 120  20], 'Style', 'slider', 'String', 'tout', 'Min', 0, 'Max', 2.0, 'Value', settings.solver.tracker.timeout, 'Callback', @sub_solver_settings_cb); 
controls.tlabel = uicontrol(f, 'Position', [10 315  60  30], 'Style', 'text', 'String', 'tout');
controls.bounce = uicontrol(f, 'Position', [70 290 120  20], 'Style', 'slider', 'String', 'bounce', 'Min', 0, 'Max', 0.5, 'Value', settings.solver.bounce.dt, 'Callback', @sub_solver_settings_cb); 
controls.blabel = uicontrol(f, 'Position', [10 285  60  30], 'Style', 'text', 'String', 'bounce');
% settings.solver.perrobot          =  1;      % solve per robot individually?

% plot filters
for irobot = 1:6
    controls.rsel{irobot}   = uicontrol(f, 'Style', 'checkbox', 'Value', settings.measfilter.robots(irobot), 'Position', [30*irobot-10 235 20 20], 'Callback', {@sub_checkbox_cb, irobot});
    controls.rlabel{irobot} = uicontrol(f, 'Position', [30*irobot-10 250 20 20], 'Style', 'text', 'String', sprintf('r%d', irobot));
end
fig3checkboxes  = fieldnames(settings.visualizer.fig3);
for icheckbox = 1:numel(fig3checkboxes)
    lbl1 = fig3checkboxes{icheckbox};
    lbl2 = lbl1(1:(min(end,3)));
    controls.fig3{icheckbox}    = uicontrol(f, 'Position', [30*icheckbox-10 185 20 20], 'Style', 'checkbox', 'Value', getfield(settings.visualizer.fig3, lbl1), 'Callback', {@sub_checkbox3_cb, lbl1});
    controls.f3label{icheckbox} = uicontrol(f, 'Position', [30*icheckbox-10 200 20 20], 'Style', 'text', 'String', lbl2);
end
fig2checkboxes  = fieldnames(settings.visualizer.fig2);
% first row
OFFSET = 7;
for icheckbox = 1:OFFSET
    lbl1 = fig2checkboxes{icheckbox};
    lbl2 = lbl1(1:(min(end,3)));
    controls.fig2{icheckbox}    = uicontrol(f, 'Position', [30*icheckbox-10 135 20 20], 'Style', 'checkbox', 'Value', getfield(settings.visualizer.fig2, lbl1), 'Callback', {@sub_checkbox2_cb, lbl1});
    controls.f2label{icheckbox} = uicontrol(f, 'Position', [30*icheckbox-10 150 20 20], 'Style', 'text', 'String', lbl2);
end
% second row
for icheckbox = (OFFSET+1):numel(fig2checkboxes)
    lbl1 = fig2checkboxes{icheckbox};
    lbl2 = lbl1(1:(min(end,3)));
    controls.fig2{icheckbox}    = uicontrol(f, 'Position', [30*(icheckbox-OFFSET)-10  85 20 20], 'Style', 'checkbox', 'Value', getfield(settings.visualizer.fig2, lbl1), 'Callback', {@sub_checkbox2_cb, lbl1});
    controls.f2label{icheckbox} = uicontrol(f, 'Position', [30*(icheckbox-OFFSET)-10 100 20 20], 'Style', 'text', 'String', lbl2);
end

% busy message
controls.qlabel = uicontrol(f, 'Position', [10 10 100 30], 'Style', 'text', 'String', 'ready', 'HorizontalAlignment', 'left');

% number of balls
controls.klabel = uicontrol(f, 'Position', [90 10 160 30], 'Style', 'text', 'String', '', 'HorizontalAlignment', 'left');

% filter measurements, update some widgets
guidata(f, sub_init(settings, measurements, ax, controls, scene));
sub_filter_measurements;

% make figure visible after adding all components
set(f, 'Visible', 'on');

% update data & solve
sub_refresh(1);



function data = sub_init(settings, measurements, ax, controls, scene)
    data.settings  = settings;
    data.measurements_orig = measurements; % backup for filtering
    data.measurements = measurements; % filtering result, cache for solver
    data.axes      = ax;
    data.controls  = controls;
    if isstruct(scene)
        data.scene = scene; % simulated scene, not to be confused with mainwindow labeled '3D scene'
    end
    data.t0        = min(data.measurements_orig.data(:, fbt_meas_idx('t')));

function sub_progress_callback(v)
    assert(v >= 0.0);
    assert(v <= 1.0);
    data = sub_get_data();
    set(data.controls.qlabel, 'String', sprintf('BUSY (%d%%)', round(100*v)));
    drawnow;
    
function sub_recalculate()
    data = sub_get_data();
    % solve if requested (checkbox)
    % TODO: cache, only recalculate if needed (this is computationally expensive)
    if data.settings.visualizer.fig2.solutions
        data.solutions = fbt_solve_sequence(data.settings, data.measurements, '-pc', @sub_progress_callback);
        % filter solutions - TODO make control buttons
        data.solutions = fbt_sol_filter(data.solutions);
        % analyze result
        extra_args     = {};
        if isfield(data, 'scene')
            extra_args = [extra_args '-scene', data.scene];
        end
        data.kpi       = fbt_kpi(data.settings, data.measurements, data.solutions, extra_args{:});
    else
        if isfield(data, 'solutions')
            data = rmfield(data, 'solutions');
        end
    end
    guidata(gcf, data);

function sub_redraw()
    data = sub_get_data();
    % figure 1: 3d scene
    % TODO options: connect, speedfactor
    cla(data.axes{1});
    fbt_meas_plot(data.measurements, '-axes', data.axes{1});
    if isfield(data, 'solutions')
        fbt_sol_plot(data.solutions, '-axes', data.axes{1});
    end
    axis(data.axes{1}, 'equal');
    % figure 2: ball in FCS + solution
    cla(data.axes{2});
    sf = data.settings.visualizer.fig2;
    whatlist = [sf.bx, sf.by, sf.bz];
    fbt_meas_plot(data.measurements, '-mode', 2, '-axes', data.axes{2}, '-what', whatlist, '-t0', data.t0, '-textlabels');
    text_y_offset = 0.1;
    % solutions in figure 2
    if isfield(data, 'solutions')
        t = data.solutions.t - data.t0;
        whatlist = {{t, data.solutions, 'sol', 'k'}};
        if sf.filter
            % plot INSTEAD, not in addition to 'raw' solutions
            whatlist = {{data.kpi.filtered.t, data.kpi.filtered.ma, 'ma', 'g'}};
        end
        if sf.simulated && isfield(data.kpi, 'ballsim')
            whatlist{end+1} = {data.kpi.filtered.t, data.kpi.ballsim, 'sim', 'g'};
        end
        if sf.errors && isfield(data.kpi, 'errors')
            whatlist{end+1} = {data.kpi.filtered.t, data.kpi.errors, 'err', 'r'};
        end
        for iwhat = 1:numel(whatlist)
            t        = whatlist{iwhat}{1};
            dt       = (t * 0) + (1.0 / data.settings.solver.ana.frequency);
            plotwhat = whatlist{iwhat}{2};
            leg      = whatlist{iwhat}{3};
            col      = [whatlist{iwhat}{4} 'o'];
            if sf.position
                if sf.bx
                    quiver(data.axes{2}, t, plotwhat.x, dt, dt.*plotwhat.vx, 0, col);
                    tt = text(t(end), plotwhat.x(end) + text_y_offset, [leg '.x']);
                    set(tt, 'Parent', data.axes{2});
                end
                if sf.by
                    quiver(data.axes{2}, t, plotwhat.y, dt, dt.*plotwhat.vy, 0, col);
                    tt = text(t(end), plotwhat.y(end) + text_y_offset, [leg '.y']);
                    set(tt, 'Parent', data.axes{2});
                end
                if sf.bz
                    quiver(data.axes{2}, t, plotwhat.z, dt, dt.*plotwhat.vz, 0, col);
                    tt = text(t(end), plotwhat.z(end) + text_y_offset, [leg '.z']);
                    set(tt, 'Parent', data.axes{2});
                end
            end
            if sf.velocity
                if sf.bx
                    plot(data.axes{2}, t, plotwhat.vx, col);
                    tt = text(t(end), plotwhat.vx(end) + text_y_offset, [leg '.vx']);
                    set(tt, 'Parent', data.axes{2});
                end
                if sf.by
                    plot(data.axes{2}, t, plotwhat.vy, col);
                    tt = text(t(end), plotwhat.vy(end) + text_y_offset, [leg '.vy']);
                    set(tt, 'Parent', data.axes{2});
                end
                if sf.bz
                    plot(data.axes{2}, t, plotwhat.vz, col);
                    tt = text(t(end), plotwhat.vz(end) + text_y_offset, [leg '.vz']);
                    set(tt, 'Parent', data.axes{2});
                end
            end
        end
    end
    % figure 3: raw measurements
    cla(data.axes{3});
    sf = data.settings.visualizer.fig3;
    whatlist = [sf.az, sf.el, sf.r, sf.conf, sf.cx, sf.cy, sf.cphi];
    fbt_meas_plot(data.measurements, '-mode', 3, '-axes', data.axes{3}, '-what', whatlist, '-t0', data.t0, '-textlabels');

function data = sub_get_data()
    % wrapper around guidata, but mainly to make any callback trigger BUSY state
    data = guidata(gcf);
    set(data.controls.qlabel, 'String', 'BUSY');
    drawnow;
    % note that refresh ends with clearing the BUSY state
    
function sub_refresh(do_recalculate)
    data = sub_get_data();
    set(data.controls.wlabel, 'String', sprintf('weight:\n%.3f', data.settings.solver.core.weight));
    set(data.controls.flabel, 'String', sprintf('freq:\n%.1f', data.settings.solver.ana.frequency));
    set(data.controls.alabel, 'String', sprintf('agesolve:\n%.3f', data.settings.solver.bounce.age));
    set(data.controls.tlabel, 'String', sprintf('timeout:\n%.3f', data.settings.solver.tracker.timeout));
    set(data.controls.blabel, 'String', sprintf('tbounce:\n%.3f', data.settings.solver.bounce.dt));
    if do_recalculate
        sub_recalculate;
    end
    data = sub_get_data();
    sub_redraw;
    set(data.controls.qlabel, 'String', 'ready');
    if isfield(data, 'solutions')
        numballs = max(data.solutions.n);
        if numballs == 1
            kpistr = sprintf('%d ball', numballs);
        else
            kpistr = sprintf('%d balls', numballs);
        end
        if isfield(data, 'kpi')
            % if present, show errors against simulation
            if isfield(data.kpi, 'errors')
                errpos = max([max(abs(data.kpi.errors.x)), max(abs(data.kpi.errors.y)), max(abs(data.kpi.errors.z))]);
                errvel = max([max(abs(data.kpi.errors.vx)), max(abs(data.kpi.errors.vy)), max(abs(data.kpi.errors.vz))]);
                kpistr = [kpistr sprintf(' (ep=%5.2f, ev=%5.2f)', errpos, errvel)];
                kpistr = [kpistr sprintf('\n(sp=%5.2f, sv=%5.2f)', data.kpi.stab.pos, data.kpi.stab.vel)];
            else
                % no simulation, so show noise levels
                kpistr = [kpistr sprintf(' (sp=%5.2f, sv=%5.2f)', data.kpi.stab.pos, data.kpi.stab.vel)];
            end
        end
        % stddev on position and velocity
        set(data.controls.klabel, 'String', kpistr);
    end

function sub_filter_measurements()
    data = sub_get_data();
    % filter on time
    % TODO: when zooming in, it would be more efficient to take measurements as starting point instead of measurements_orig
    meas_allrobots = fbt_meas_filter(data.measurements_orig, '-tmin', data.settings.measfilter.tmin, '-tmax', data.settings.measfilter.tmax);
    % filter on robots
    data.measurements = [];
    for irobot = find(data.settings.measfilter.robots)
        data.measurements = fbt_meas_merge(data.measurements, fbt_meas_filter(meas_allrobots, '-robot', irobot));
    end
    % by design of sub_filter_measurements_cb, it is impossible that tmin > tmax 
    % make sure tcurr pointer is within limits
    if data.settings.measfilter.tcurr > data.settings.measfilter.tmax
        data.settings.measfilter.tcurr = data.settings.measfilter.tmax;
    end
    if data.settings.measfilter.tcurr < data.settings.measfilter.tmin
        data.settings.measfilter.tcurr = data.settings.measfilter.tmin;
    end
    data.controls.tcur.Value = data.settings.measfilter.tcurr;
    % update labels to print time values
    t = data.settings.measfilter.tmin;
    data.controls.tmin_lab.String = sprintf('tmin (%10.6f / %s)', t - data.t0, fbt_time_float2str(t));
    t = data.settings.measfilter.tmax;
    data.controls.tmax_lab.String = sprintf('tmax (%10.6f / %s)', t - data.t0, fbt_time_float2str(t));
    t = data.settings.measfilter.tcurr;
    data.controls.tcur_lab.String = sprintf('tcurr (%10.6f / %s)', t - data.t0, fbt_time_float2str(t));
    guidata(gcf, data);

function sub_checkbox_cb(source, callbackdata, irobot)
    data = sub_get_data();
    data.settings.measfilter.robots(irobot) = get(source, 'Value');
    guidata(gcf, data);
    sub_filter_measurements;
    sub_refresh(1);
    
function sub_checkbox2_cb(source, callbackdata, fn)
    data = sub_get_data();
    data.settings.visualizer.fig2 = setfield(data.settings.visualizer.fig2, fn, get(source, 'Value'));
    guidata(gcf, data);
    do_recalculate = strcmp(fn, 'solutions'); % only recalculate upon toggle
    sub_refresh(do_recalculate);
    
function sub_checkbox3_cb(source, callbackdata, fn)
    data = sub_get_data();
    data.settings.visualizer.fig3 = setfield(data.settings.visualizer.fig3, fn, get(source, 'Value'));
    guidata(gcf, data);
    sub_refresh(0);
    
function sub_filter_measurements_cb(source, callbackdata)
    data = sub_get_data();
    if strcmp(get(source, 'String'), 'tmax')
        if data.settings.measfilter.tmin < get(source, 'Value')
            data.settings.measfilter.tmax = get(source, 'Value');
        end
    end
    if strcmp(get(source, 'String'), 'tmin')
        if data.settings.measfilter.tmax > get(source, 'Value')
            data.settings.measfilter.tmin = get(source, 'Value');
        end
    end
    if strcmp(get(source, 'String'), 'tcurr')
        if (get(source, 'Value') < data.settings.measfilter.tmax) && ...
            (get(source, 'Value') > data.settings.measfilter.tmin)
            data.settings.measfilter.tcurr = get(source, 'Value');
        end
    end
    guidata(gcf, data);
    sub_filter_measurements;
    sub_refresh(1);
    
function sub_solver_settings_cb(source, callbackdata)
    data = sub_get_data();
    if strcmp(get(source, 'String'), 'weight')
        data.settings.solver.core.weight = get(source, 'Value');
    end
    if strcmp(get(source, 'String'), 'freq')
        data.settings.solver.ana.frequency = get(source, 'Value');
    end
    if strcmp(get(source, 'String'), 'asol')
        data.settings.solver.bounce.age = get(source, 'Value');
    end
    if strcmp(get(source, 'String'), 'bounce')
        data.settings.solver.bounce.dt = get(source, 'Value');
    end
    if strcmp(get(source, 'String'), 'tout')
        data.settings.solver.tracker.timeout = get(source, 'Value');
    end
    guidata(gcf, data);
    sub_refresh(1);

function sub_toggle_withspeed_cb(source, callbackdata)
    data = sub_get_data();
    if data.settings.solver.core.speed
        data.settings.solver.core.speed = 0;
        set(source, 'String', 'enable speed');
    else
        data.settings.solver.core.speed = 1;
        set(source, 'String', 'disable speed');
    end
    guidata(gcf, data);
    sub_refresh(1);

function sub_dump_cb(source, callbackdata)
    data = sub_get_data();
    % print all kinds of things to Matlab stdout
    disp('Settings:');
    disp(repmat('-', [1 50]));
    disp(colprintf(data.settings));
    if isfield(data, 'kpi')
        disp('Key Performance Indicators:');
        disp(repmat('-', [1 50]));
        disp(colprintf(data.kpi));
    end
    % minimize window to immediately make it visible
    warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame;
    jFrame = get(handle(1), 'JavaFrame');
    jFrame.setMinimized(1);
    