function fbt_write_testdata(varargin)
% utility to modify test data for use in worldModel/tst suite
% 
% mode 1: if output file is given, it will parse it, find the octave command string, 
% evaluate it and make sure the data ends up in the file
%
% mode 2: if scene is given, it will simulate it and write (append) 
% resulting measurements as well as solutions to the file



% option parsing
options.randseed = 0;
options.tmax    = 1e20;
options.output  = 'octave_generated.txt';
options.tstdir  = '/home/robocup/falcons/code/packages/worldModel/tst/ball';
[options, args] = getopts(options, varargin{:});

% require 1 argument, either scene or filename
assert(numel(args) == 1);
if isstruct(args{1})
    if isfield(args{1}, 'solutions')
        % no need to simulate anymore
        data    = args{1};
    else
        % simulate scene
        scene   = args{1};
        data    = sub_simulate(options, scene);
    end
else
    options.output = args{1};
    sub_parse_eval(options);
    return
end

% write data
sub_write(options, data);



% sub functions below

function data = sub_simulate(options, scene)
    settings    = fbt_settings(0);
    settings.solver.conf.goodlim = -1; % disable confidence heuristic for this simulation
    settings.sim.tmax = options.tmax;
    errors      = fbt_errors(0);
    data        = fbt_sim_solve(scene, settings, errors, options.randseed);
    data.solutions = data.solution.merged;
    data.solutions.conf(:) = 0; % irrelevant in simulation

function sub_write(options, data)
    filename    = [options.tstdir filesep options.output];
    % append
    fbt_meas_write(data.measurements, filename, 'a');
    fbt_sol_write(data.solutions, filename, 'a');

function sub_parse_eval(options)
    filename    = [options.tstdir filesep options.output];
    assert(isfile(filename));
    % if needed, remove generated bottom of file
    fid         = fopen(filename, 'r');
    C           = textscan(fid,'%s','delimiter','\n');
    fclose(fid);
    lines       = C{1};
    [~, idx]    = ismember('# BELOW IS GENERATED, DO NOT EDIT', lines);
    if idx > 0
        % remove
        fid     = fopen(filename, 'w');
        for iline = 1:idx
            fprintf(fid, '%s\n', lines{iline});
        end
        fclose(fid);
    end
    % find the octave command string
    octave_cmd  = '';
    for iline = 1:idx
        if numel(strfind(lines{iline}, '# octave: '))
            octave_cmd = lines{iline}(11:end);
        end
    end    
    assert(numel(octave_cmd));
    % modify such that it writes to actual file
    octave_cmd  = [octave_cmd(1:(end-2)) ', ''-output'', ''', options.output, ''');'];
    % evaluate
    eval(octave_cmd); 

