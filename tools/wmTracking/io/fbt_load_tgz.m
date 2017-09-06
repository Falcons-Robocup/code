function data = fbt_load_tgz(filename, varargin)
% read data from a file: ball measurements + algorithm results, same for obstacles
% optimized for speed: it cannot handle tracing, instead some scripting
% utility should be used to preprocess tracing into something which we can simply load as Matlab matrix
% see also getWmTracing - the script which generates such a tgz


% no options
options.quiet       = false;
options.verbose     = false;
[options, args]     = getopts(options, varargin{:});
if options.quiet
    options.verbose = false;
else
    options.verbose = true;
end

% make sure file name is absolute
if (~isfile(filename))
    [loc, fn, ext]  = fileparts(filename);
    datafolder      = [fileparts(mfilename('fullpath')) filesep '..' filesep 'data'];
    filename        = [datafolder filesep fn ext];
end
assert(1 == isfile(filename));
filename           = fullglob(filename);
filename           = filename{1};

% logging
if options.verbose
    disp(sprintf('# loading %s ...', filename));
end

% unpack
assert(numel(strfind(filename, 'tgz')) > 0);
cwd                = pwd;
cd(tempdir);
files              = untar(filename);
need_cleanup       = true;

% process the files
assert(numel(files) == 4);
assert(ismember('ballMeasurements.txt', files));
assert(ismember('ballResults.txt', files));
assert(ismember('obstacleMeasurements.txt', files));
assert(ismember('obstacleResults.txt', files));
data.ball.measurements = sub_load_ball_measurements('ballMeasurements.txt');
data.ball.results = sub_load_ball_solutions('ballResults.txt');
data.obstacle.measurements = sub_load_ball_measurements('obstacleMeasurements.txt');
data.obstacle.results = sub_load_obstacle_solutions('obstacleResults.txt');

% set time bounds
data = sub_set_tmin_tmax(data);

% cleanup
cd(cwd);


% sub functions below

function measurements = sub_load_ball_measurements(filename)
    data = load(filename);
    measurements = fbt_meas(size(data, 1));
    % map columns
    file_columns = {'id', 'cam', 't', 'cx', 'cy', 'cz', 'cphi', 'az', 'el', 'r', 'conf'};
    assert(size(data, 2) == numel(file_columns));
    for it = 1:numel(measurements.columns)
        % determine target column
        [tf, idx] = ismember(measurements.columns{it}, file_columns);
        if tf
            measurements.data(:, it) = data(:, idx);
        else
            assert(ismember(measurements.columns{it}, {'bx', 'by', 'bz'}));
        end
    end
    % extend metadata bx,by,bz
    measurements = fbt_meas_extend(measurements);

function solutions = sub_load_ball_solutions(filename)
    data = load(filename);
    solutions = fbt_sol(size(data, 1));
    % map columns
    file_columns = {'t', 'tr', 'nb', 'id', 'conf', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'age', 'nmeas', 'ncam', 'omni', 'qcam', 'qage', 'qm', 'qfr', 'qz', 'qv', 'qfit', 'res'};
    assert(size(data, 2) == numel(file_columns));
    for it = 1:numel(solutions.columns)
        % determine target column
        [tf, idx] = ismember(solutions.columns{it}, file_columns);
        if tf
            solutions.data(:, it) = data(:, idx);
        end
    end

function solutions = sub_load_obstacle_solutions(filename)
    data = load(filename);
    solutions = fbt_sol(size(data, 1));
    % map columns
    file_columns = {'t', 'tr', 'ok', 'id', 'x', 'y', 'vx', 'vy', 'N', 'age', 'fresh'};
    assert(size(data, 2) == numel(file_columns));
    for it = 1:numel(solutions.columns)
        % determine target column
        [tf, idx] = ismember(solutions.columns{it}, file_columns);
        if tf
            solutions.data(:, it) = data(:, idx);
        end
    end

function data = sub_set_tmin_tmax(data)
    % for sliders in fbt_gui
    data.tmin = min(data.ball.measurements.data(:, fbt_col_idx('t', data.ball.measurements)));
    data.tmin = min(data.tmin, min(data.obstacle.measurements.data(:, fbt_col_idx('t', data.obstacle.measurements))));
    data.tmax = max(data.ball.measurements.data(:, fbt_col_idx('t', data.ball.measurements)));
    data.tmax = max(data.tmax, max(data.obstacle.measurements.data(:, fbt_col_idx('t', data.obstacle.measurements))));
    % for plotters
    data.ball.measurements.t0 = data.tmin;
    data.ball.results.t0 = data.tmin;
    data.obstacle.measurements.t0 = data.tmin;
    data.obstacle.results.t0 = data.tmin;
