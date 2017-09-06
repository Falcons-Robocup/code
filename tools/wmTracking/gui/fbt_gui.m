function fbt_gui(varargin)
% JFEI 2017Q1 ball&obstacle tracking analysis GUI
% example:
%    tgzdata = fbt_load_tgz('wmTracing_r4_20170211_morning_VDL.tgz');
%    close all ; fbt_gui(tgzdata)


% initialize global data
data = sub_init_data;

% initialize GUI figure
f = sub_init_gui(data);

% make the figure visible 
set(f, 'Visible', 'on');

% handle input arguments
if nargin
    sub_set_file_data(varargin{1});
end

% done!



% sub functions - data handling and initialization

function settings = sub_settings()
    settings = fbt_settings; % TODO ball versus obstacle etc etc
    settings.filter = settings.measfilter; % hack
    settings.filter.trackers = []; % hide list actually
    fn = {'t', 'omni', 'front', 'bx', 'by', 'bz'};
    for it = 1:numel(fn)
        settings.filter = setfield(settings.filter, fn{it}, 1);
    end
    fn = {'id', 'cam', 'az', 'el', 'r', 'cx', 'cy', 'cphi'};
    for it = 1:numel(fn)
        settings.filter = setfield(settings.filter, fn{it}, 0);
    end
    fn = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'conf'};
    for it = 1:numel(fn)
        settings.filter = setfield(settings.filter, fn{it}, 1);
    end
    settings.filter.conf = 0;
    settings.filter.front = 0; % for now not used, default do not show
    % z and vz are not so interesting for now, disable by default
    settings.filter.z = 0;
    settings.filter.vz = 0;
    settings.playback_dt = 0.4;
    % TODO improve consistency between settings struct and control defaults, currently a bit messy

function data = sub_get_data()
    % wrapper around guidata, but mainly to make any callback trigger BUSY state
    data = guidata(gcf);
    set(data.controls.progresslabel, 'String', 'BUSY');
    drawnow;
    % refresh should end with clearing the BUSY state
    
function data = sub_init_data()
    data.datafolder   = [fileparts(mfilename('fullpath')) filesep '..' filesep 'data'];
    data.settings     = sub_settings();
    data.datatype     = 'ball';
    data.browsemode   = 'full'; % or playback
    data.ball.measurements = [];
    data.ball.results = [];
    data.obstacle.measurements = [];
    data.obstacle.results = [];
    data.axes         = {};
    data.controls     = [];
    data.playback.active   = false;
    data.playback.ticks    = [];
    data.playback.curr_idx = 0;

function f = sub_init_gui(data)
    % create a figure and axes 
    f = figure('Visible', 'off', 'Units', 'pixels', 'Position', [5 45 1350 600], 'Name', 'Falcons Ball- and Obstacle Tracking GUI');
    data.axes{1} = axes('Units', 'pixels', 'Position', [300  50 440 390], 'Parent', f, 'Tag', 'ax1');
    data.axes{1}.Title.String = 'data over time';
    grid on;
    data.axes{2} = axes('Units', 'pixels', 'Position', [800  50 500 390], 'Parent', f, 'Tag', 'ax2');
    data.axes{2}.Title.String = '3D plot';
    grid on;
    % add controls
    data.controls = sub_create_main_controls(f, data.settings);
    % store guidata
    guidata(gcf, data);
    
function sub_load_dialog_cb(source, callbackdata)
    data = sub_get_data();
    cd(data.datafolder);
    [filename, pathname] = uigetfile('*.tgz', 'select a worldModel tracing .tgz file');
    filedata = fbt_load_tgz([pathname filesep filename]);
    sub_set_file_data(filedata);
    
function sub_set_file_data(filedata)
    data = sub_get_data();
    data.ball = filedata.ball;
    data.obstacle = filedata.obstacle;
    data.settings.filter.tmin = filedata.tmin;
    data.settings.filter.tmax = filedata.tmax;
    data.settings.filter.tcurr = filedata.tmin;
    data.t0 = filedata.tmin;
    data.te = filedata.tmax;
    % determine playback solver ticks
    solutions = data.ball.results;
    t1 = solutions.data(:, fbt_col_idx('t', solutions));
    solutions = data.obstacle.results;
    t2 = solutions.data(:, fbt_col_idx('t', solutions));
    data.playback.ticks = unique(sort([t1; t2]));
    guidata(gcf, data);
    sub_init_sliders_toggles();
    sub_refresh(0);

% sub functions - solver

function sub_progress_callback(v)
    assert(v >= 0.0);
    assert(v <= 1.0);
    data = sub_get_data();
    set(data.controls.progresslabel, 'String', sprintf('BUSY (%d%%)', round(100*v)));
    drawnow;

function sub_refresh(do_recalculate)
    data = sub_get_data();
    if do_recalculate
        sub_recalculate;
    end
    sub_redraw;
    set(data.controls.progresslabel, 'String', 'ready');

function sub_recalculate()
    data = sub_get_data();
    % solve if requested 
%     if data.settings.solve
%         data.solutions = fbt_solve_sequence(data.settings, data.measurements, '-pc', @sub_progress_callback);
%     else
%         if isfield(data, 'solutions')
%             data = rmfield(data, 'solutions');
%         end
%     end
    guidata(gcf, data);
    
% sub functions - GUI control layout & callbacks
    
function controls = sub_create_main_controls(f, settings)
    % top level time sliders and labels
    controls.time.sliders.min      = uicontrol(f, 'Position', [600 575  700  20], 'Style', 'slider', 'String', 'tmin', 'Min', 0, 'Max', 1, 'Value', 1, 'Callback', @sub_filter_measurements_cb); 
    controls.time.sliders.max      = uicontrol(f, 'Position', [600 550  700  20], 'Style', 'slider', 'String', 'tmax', 'Min', 0, 'Max', 1, 'Value', 1, 'Callback', @sub_filter_measurements_cb); 
    controls.time.sliders.curr     = uicontrol(f, 'Position', [600 525  700  20], 'Style', 'slider', 'String', 'tcurr', 'Min', 0, 'Max', 1, 'Value', 0, 'Callback', @sub_filter_measurements_cb); 
    controls.time.labels.min       = uicontrol(f, 'Position', [260 572  330  20], 'Style', 'text', 'String', 'tmin', 'FontName', 'FixedWidth', 'HorizontalAlignment', 'right');
    controls.time.labels.max       = uicontrol(f, 'Position', [260 547  330  20], 'Style', 'text', 'String', 'tmax', 'FontName', 'FixedWidth', 'HorizontalAlignment', 'right');
    controls.time.labels.curr      = uicontrol(f, 'Position', [260 522  330  20], 'Style', 'text', 'String', 'tcurr', 'FontName', 'FixedWidth', 'HorizontalAlignment', 'right');
    % button section
    controls.main.buttons.load     = uicontrol(f, 'Position', [10  570  70  30], 'String', 'load data', 'Tag', 'button', 'Callback', @sub_load_dialog_cb);
    controls.main.buttons.mode     = uicontrol(f, 'Position', [100 570  70  30], 'String', 'switch mode', 'Tag', 'button', 'Callback', @sub_switch_mode_cb);
    controls.main.labels.mode      = uicontrol(f, 'Position', [170 570  100  20], 'Style', 'text', 'String', 'current=BALL');
    controls.main.buttons.settings = uicontrol(f, 'Position', [10  530  70  30], 'String', 'settings', 'Tag', 'button', 'Callback', @sub_settings_dialog_cb);
    controls.main.buttons.model    = uicontrol(f, 'Position', [100 530  70  30], 'String', 'model', 'Tag', 'button', 'Callback', @sub_model_cb);
    controls.main.buttons.play     = uicontrol(f, 'Position', [10  490  70  30], 'String', 'play', 'Tag', 'button', 'Callback', @sub_play_cb);
    controls.main.buttons.reset    = uicontrol(f, 'Position', [100 490  70  30], 'String', 'reset', 'Tag', 'button', 'Callback', @sub_reset_cb);
    % some settings & spacers
    rowheight = 45;
    xs        = 30;
    ys1       = 15;
    ox        = 0;
    baserow   = 460;
    label_properties = {'HorizontalAlignment', 'left', 'Style', 'text', 'String'}; % next one should be the label id
    % plot filters - trackers (updated live during playback, so here we only need to initialize empty)
    controls.filt.trsel = {};
    controls.filt.trlabel = {};
    % plot filters - measurements sources
    irow = 1; h = baserow - rowheight * irow;
    uicontrol(f, 'Position', [10 h+20 200 20], 'FontWeight', 'bold', label_properties{:}, 'measurement filters');
    for irobot = 1:6
        controls.filt.rsel{irobot}   = uicontrol(f, 'Style', 'checkbox', 'Value', settings.filter.robots(irobot), 'Position', [xs*irobot-10 h-ys1 20 20], 'Callback', {@sub_checkbox_cb, irobot});
        controls.filt.rlabel{irobot} = uicontrol(f, 'Position', [xs*irobot-10+ox h 20 20], label_properties{:}, sprintf('r%d', irobot));
    end
    irow = 2; h = baserow - rowheight * irow;
    controls.filt.omni             = uicontrol(f, 'Style', 'checkbox', 'Value', 1, 'Position', [xs*1-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, 'omni'});
    controls.filt.front            = uicontrol(f, 'Style', 'checkbox', 'Value', 1, 'Position', [xs*2-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, 'front'});
    controls.filt.labels.omni      = uicontrol(f, 'Position', [xs*1-10+ox h 40 20], label_properties{:}, 'omni');
    controls.filt.labels.front     = uicontrol(f, 'Position', [xs*2-10+ox h 40 20], label_properties{:}, 'front');
    % plot filters - metadata is most interesting, because comparable to solution
    irow = 2; h = baserow - rowheight * irow;
    elements = {'bx', 'by', 'bz'};
    for it = 1:numel(elements)
        lbl = elements{it};
        it2 = it+3;
        controls.filt.(lbl)        = uicontrol(f, 'Style', 'checkbox', 'Value', 1, 'Position', [xs*it2-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, lbl});
        controls.filt.labels.(lbl) = uicontrol(f, 'Position', [xs*it2-10+ox h 20 20], label_properties{:}, lbl);
    end
    % plot filters - raw measurement elements, default hide
    irow = 3; h = baserow - rowheight * irow;
    elements = {'az', 'el', 'r', 'cx', 'cy', 'cphi'};
    for it = 1:numel(elements)
        lbl = elements{it};
        controls.filt.(lbl)        = uicontrol(f, 'Style', 'checkbox', 'Value', 0, 'Position', [xs*it-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, lbl});
        controls.filt.labels.(lbl) = uicontrol(f, 'Position', [xs*it-10+ox h 50 20], label_properties{:}, lbl);
    end
    % plot filters - robot solver results
    irow = 5; h = baserow - rowheight * irow;
    uicontrol(f, 'Position', [10 h+20 200 20], 'FontWeight', 'bold', label_properties{:}, 'robot tracking results');
    elements = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'conf'};
    for it = 1:numel(elements)
        lbl = elements{it};
        controls.filt.(lbl)        = uicontrol(f, 'Style', 'checkbox', 'Value', 1, 'Position', [xs*it-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, lbl});
        controls.filt.labels.(lbl) = uicontrol(f, 'Position', [xs*it-10+ox h 50 20], label_properties{:}, lbl);
    end
    % offline solver results and error, default disabled
    irow = 7; h = baserow - rowheight * irow;
    uicontrol(f, 'Position', [10 h+20 200 20], 'FontWeight', 'bold', label_properties{:}, 'offline solver tracking results');
    elements = {'sx', 'sy', 'sz', 'svx', 'svy', 'svz', 'sconf'};
    for it = 1:numel(elements)
        lbl = elements{it};
        controls.filt.(lbl)        = uicontrol(f, 'Style', 'checkbox', 'Value', 0, 'Position', [xs*it-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, lbl});
        controls.filt.labels.(lbl) = uicontrol(f, 'Position', [xs*it-10+ox h 50 20], label_properties{:}, lbl);
    end
    irow = 8; h = baserow - rowheight * irow;
    elements = {'ex', 'ey', 'ez', 'evx', 'evy', 'evz'};
    for it = 1:numel(elements)
        lbl = elements{it};
        controls.filt.(lbl)        = uicontrol(f, 'Style', 'checkbox', 'Value', 0, 'Position', [xs*it-10 h-ys1 20 20], 'Callback', {@sub_filter_cb, lbl});
        controls.filt.labels.(lbl) = uicontrol(f, 'Position', [xs*it-10+ox h 50 20], label_properties{:}, lbl);
    end
    % busy message
    controls.progresslabel = uicontrol(f, 'Position', [10 10 100 30], 'Style', 'text', 'String', 'ready', 'HorizontalAlignment', 'left');

function s = sub_tstring(desc, t, t0)
    ts = fbt_time_float2str(t); 
    ts = ts(12:23);
    t2 = t - floor(t0 / 1000) * 1000;
    s = sprintf('%s (%8.3f / %s / %8.3f)', desc, t - t0, ts, t2);
    
function sub_update_time_lables()
    data = sub_get_data();
    t0 = data.t0;
    te = data.te;
    t = data.settings.filter.tmin; 
    data.controls.time.sliders.min.Value = (t - t0) / (te - t0);
    data.controls.time.labels.min.String = sub_tstring('tmin', t, t0);
    t = data.settings.filter.tmax; 
    data.controls.time.sliders.max.Value = (t - t0) / (te - t0);
    data.controls.time.labels.max.String = sub_tstring('tmax', t, t0);
    t = data.settings.filter.tcurr; 
    data.controls.time.sliders.curr.Value = (t - t0) / (te - t0);
    data.controls.time.labels.curr.String = sub_tstring('tcurr', t, t0);
    guidata(gcf, data);

function sub_init_sliders_toggles()
    data = sub_get_data();
    sub_update_time_lables();
    measurements = data.(data.datatype).measurements;
    solutions = data.(data.datatype).results;
    idx = fbt_col_idx('cam', measurements);
    data.controls.filt.front.Value = any(measurements.data(:, idx) == 2);
    idx = fbt_col_idx('id', measurements);
    for irobot = 1:6
        data.controls.filt.rsel{irobot}.Value = any(measurements.data(:, idx) == irobot);
    end
    guidata(gcf, data);

% sub functions - update figure 

function sub_redraw()
    data = sub_get_data();
    % browse mode or playback mode?
    if strcmp(data.browsemode, 'playback')
        t = data.playback.ticks(data.playback.curr_idx);
        sub_plot_time_tick(t);
    else
        % get original data
        measurements = data.(data.datatype).measurements;
        solutions = data.(data.datatype).results;
        sub_update_figures(measurements, solutions, data.axes, data.settings);
    end
    
function sub_plot_field_features(h)
    % green color
    attrs = {'LineWidth', 1, 'Color', [0 1 0]};
    % field boundaries and middle line
    x = 6;
    y = 9;
    ll = line([-x -x x x -x -x x], [0 -y -y y y 0 0], [0 0 0 0 0 0 0], attrs{:});
    % center circle
    ang = 0:0.01:2*pi; 
    r = 2;
    xp = r * cos(ang);
    yp = r * sin(ang);
    line(xp, yp, xp*0, attrs{:});

function sub_update_figures(measurements, solutions, axes, settings)
    cla(axes{1});
    cla(axes{2});
    % field base
    sub_plot_field_features(axes{2});
    % select only what needs to be drawn
    % output1 is fully filtered with checkboxes, but output2 is filtered only
    % on time refilter and robots because it does not make sense to plot only partial coordinates
    [meas1, meas2] = sub_filter_measurements(measurements, settings);
    [sol1, sol2] = sub_filter_solutions(solutions, settings);
    % plot in left window (timed)
    fbt_meas_plot_time(meas1, '-reusefig', '-axes', axes{1}, '-textlabels');
    fbt_sol_plot_time(sol1, '-reusefig', '-axes', axes{1}, '-textlabels');
    % plot in right window (3D)
    fbt_meas_plot_3d(meas2, '-axes', axes{2});
    fbt_sol_plot_3d(sol2, '-axes', axes{2});
    % in playback mode, set axis tight
    if size(meas1.data, 1) < 500 % TODO: improve this poor way to check...
        v = axis(axes{1});
        t = settings.filter.tcurr - measurements.t0;
        v(1) = t - 1.3;
        v(2) = t + 0.3;
        v(3) = -10;
        v(4) = 10;
        axis(axes{1}, v);
    end
    % fixate the camera for 3d scene
    v = [-7 7 -10 10 -1 4];
    axis(axes{2}, v);
    campos(axes{2}, [0 0 45]); % campos(axes{2}, [76 -20 8]);

function [meas1, meas2] = sub_filter_measurements(meas_in, settings)
    % filter on time
    % TODO: when zooming in, it would be more efficient to take measurements as starting point instead of measurements_orig
    meas_allrobots = fbt_filter_time(meas_in, settings.filter.tmin, settings.filter.tmax);
    % filter on robots
    meas_out = fbt_filter_select(meas_allrobots, 'id', find(settings.filter.robots));
    % filter on cam
    settings.filter.cams = find([settings.filter.omni settings.filter.front]);
    meas_out = fbt_filter_select(meas_out, 'cam', settings.filter.cams);
    meas2 = meas_out;
    % deselect data columns which user does not want to see
    datacolumns = {'az', 'el', 'r', 'cx', 'cy', 'cphi', 'bx', 'by', 'bz'}; % measurement confidence not yet used + conflicts with solver confidence
    for it = numel(datacolumns):-1:1
        if ~getfield(settings.filter, datacolumns{it})
            datacolumns(it) = [];
        end
    end
    datacolumns = [datacolumns 't' 'id'];
    meas1 = fbt_filter_columns(meas_out, datacolumns);

function [sol1, sol2] = sub_filter_solutions(sol_in, settings)
    % filter on time
    % TODO: when zooming in, it would be more efficient to take measurements as starting point instead of measurements_orig
    sol2 = fbt_filter_time(sol_in, settings.filter.tmin, settings.filter.tmax);
    % deselect data columns which user does not want to see
    datacolumns = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'conf'};
    for it = numel(datacolumns):-1:1
        if ~getfield(settings.filter, datacolumns{it})
            datacolumns(it) = [];
        end
    end
    datacolumns = [datacolumns 't'];
    sol1 = fbt_filter_columns(sol2, datacolumns);

function sub_checkbox_cb(source, callbackdata, irobot)
    data = sub_get_data();
    data.settings.filter.robots(irobot) = get(source, 'Value');
    guidata(gcf, data);
    sub_refresh(1);
    
function sub_filter_cb(source, callbackdata, fn)
    data = sub_get_data();
    data.settings.filter.(fn) = get(source, 'Value');
    guidata(gcf, data);
    sub_refresh(0);

function t = sub_scale_time(data, tf)
    t = tf * (data.te - data.t0) + data.t0;
    
function sub_filter_measurements_cb(source, callbackdata)
    data = sub_get_data();
    % update
    data.settings.filter = setfield(data.settings.filter, get(source, 'String'), sub_scale_time(data, get(source, 'Value')));
    % clip
    data.settings.filter.tmax = min(data.settings.filter.tmax, data.te);
    data.settings.filter.tmax = max(data.settings.filter.tmax, data.t0);
    data.settings.filter.tmin = max(data.settings.filter.tmin, data.t0);
    data.settings.filter.tmin = min(data.settings.filter.tmin, data.te);
    data.settings.filter.tcurr = max(data.settings.filter.tcurr, data.settings.filter.tmin);
    data.settings.filter.tcurr = min(data.settings.filter.tcurr, data.settings.filter.tmax);
    % in playback mode, update index according to tcurr
    if strcmp(data.browsemode, 'playback')
        data.playback.curr_idx = max(find(data.playback.ticks < data.settings.filter.tcurr));
    end
    guidata(gcf, data);
    % update labels to print time values
    sub_update_time_lables();
    %sub_refresh(1); % TODO: in playback mode do not refresh, otherwise do
    
function sub_switch_mode_cb(source, callbackdata)
    data = sub_get_data();
    if strcmp(data.datatype, 'ball')
        data.datatype         = 'obstacle';
        set(data.controls.main.labels.mode, 'String', 'current=OBST');
        data.controls.filt.z.Value = 0;
        data.controls.filt.vz.Value = 0;
        data.settings.filter.z = 0;
        data.settings.filter.vz = 0;
    else
        data.datatype         = 'ball';
        set(data.controls.main.labels.mode, 'String', 'current=BALL');
    end
    guidata(gcf, data);
    sub_refresh(1);

% sub functions - playback and modeling

function sub_play_cb(source, callbackdata)
    data = guidata(gcf);
    start_thread = false;
    data.browsemode = 'playback';
    if strcmp(get(source, 'String'), 'play')
        start_thread = true;
        data.playback.active = true;
        set(source, 'String', 'pause');
    elseif strcmp(get(source, 'String'), 'pause')
        data.playback.active = false;
        set(source, 'String', 'play');
    end
    guidata(gcf, data);
    % start thread?
    if start_thread
        sub_playback_thread;
    end

function sub_playback_thread()
    data = guidata(gcf);
    while data.playback.active
        % sleep
        pause(data.settings.playback_dt);
        % update
        data = guidata(gcf); 
        data.playback.curr_idx = data.playback.curr_idx + 1;
        guidata(gcf, data);
        t = [];
        try
            t = data.playback.ticks(data.playback.curr_idx);
        end
        if isempty(t)
            break;
        end
        % plot
        sub_plot_time_tick(t);
        set(data.controls.progresslabel, 'String', 'playing');
    end
    set(data.controls.progresslabel, 'String', 'ready');

function sub_checkbox_tracker_cb(source, callbackdata, itracker)
    data = sub_get_data();
    if get(source, 'Value')
        % unhide if present
        idx = find(data.settings.filter.trackers == itracker);
        data.settings.filter.trackers(idx) = [];
    else
        % hide by adding to filter list
        data.settings.filter.trackers(end+1) = itracker;
    end
    guidata(gcf, data);
    sub_refresh(1);

function sub_update_tracker_filter_labels(solutions)
    f = gcf;
    data = guidata(f);
    % delete all current ones
    tracker_ids_filter = data.settings.filter.trackers;
    for it = numel(data.controls.filt.trsel):-1:1
        delete(data.controls.filt.trsel{it});
        delete(data.controls.filt.trlabel{it});
    end
    % set new ones
    label_properties = {'HorizontalAlignment', 'right', 'Style', 'text', 'String'}; % next one should be the label id
    y  = 480;
    y2 = y-3;
    xs = 25;
    xw = 60;
    x  = 430;
    uicontrol(f, 'Position', [x-150 y2 120 20], 'FontWeight', 'bold', label_properties{:}, 'tracker selection:');
    tracker_ids_available = unique(solutions.data(:, fbt_col_idx('id', solutions)));
    for it = 1:numel(tracker_ids_available)
        tracker_id = tracker_ids_available(it);
        xx = x+xw*(it-1);
        data.controls.filt.trsel{it}   = uicontrol(f, 'Style', 'checkbox', 'Value', ~ismember(tracker_id, tracker_ids_filter), 'Position', [xx+xs y 20 20], 'Callback', {@sub_checkbox_tracker_cb, tracker_id});
        data.controls.filt.trlabel{it} = uicontrol(f, 'Position', [xx-10 y2 30 20], label_properties{:}, sprintf('%d', tracker_id));
    end
    % TODO row-mixed-checkbox all/none
    % TODO also provide a tickbox to plot all measurements which are associated with invalid trackers 
    guidata(gcf, data);

function sub_plot_time_tick(t)
    data = guidata(gcf);
    % update tcurr slider
    data.settings.filter.tmax = data.te;
    data.settings.filter.tmin = data.t0;
    data.settings.filter.tcurr = t;
    guidata(gcf, data);
    sub_update_time_lables();
    % get original data (TODO cache some for speed?)
    measurements = data.(data.datatype).measurements;
    solutions = data.(data.datatype).results;
    settings = data.settings;
    % re-filter
    settings.filter.tmin = t - 1.0; % TODO use actual timeout parameter 
    settings.filter.tmax = t + eps;
    % only last solution
    % TODO also show recent solutions in grey? for comparison / stability
    solutions = fbt_filter_time(solutions, t - eps, t + eps);
    sub_update_tracker_filter_labels(solutions); % must be before filter
    % select trackers
    idx = fbt_col_idx('id', solutions);
    trackers_keep = setdiff(unique(solutions.data(:, idx)), data.settings.filter.trackers);
    solutions = fbt_filter_select(solutions, 'id', trackers_keep);
    % apply filters and plot
    sub_update_figures(measurements, solutions, data.axes, settings);
    
    