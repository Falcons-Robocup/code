%% Helper function to parse input arguments for options 
%
%   [ALL_OPTS, ARGS, ISSET] = getopts(DEFAULT_OPTS, ARG1, ARG2, ...., ARGN)
%       Determine options from input arguments ARG1 to ARGN and return a
%       structure with a complete set of options.
%       GETOPTS is for filtering argument strings starting with '-'. 
%
%       An input argument can be:
%           - regular input argument
%           - option name, starting with '-'
%           - option value, belonging to an option name
%       Options can occur as a two argument name/value pair or as one 
%       argument with only an option name. 
%       The option value always directly follows the option name.
%       An option name without a value, is called a boolean option.
%
%       Options may occur in any particular order, except that an option
%       value must directly follow the option name.  Option names are case
%       sensitive and may be truncated to an unambiguous option name.
%
%       DEFAULT_OPTS is a structure with fields that specify all possible
%       option names.  The names of the fields are identical to the names
%       of the options.  The value of the fields specify the default value
%       for each option.  This default can be any valid Matlab variable.  
%       An option is defined to be a boolean option when the default value
%       is a logical FALSE. Note that 0 is not the same as FALSE.
%
%       ALL_OPTS is a structure that has the same fields as DEFAULT_OPTS.
%       The values of the fields in ALL_OPTS are either filled with the
%       value from the corresponding option name/value pairs, or, when not
%       specified, filled with the values from DEFAULT_OPTS. When an option 
%       value is empty, the default value from DEFAULTS_OPTS will be set. 
%       If a field in DEFAULT_OPTS is FALSE (indicating it is a boolean 
%       option without a value), then the corresponding field in ALL_OPTS 
%       will be set to TRUE.
%       When an option occurs more than once in the input arguments, only
%       the last one will take effect.
%
%       ARGS is a cell array with all regular arguments. It cantains all 
%       input arguments in the same order, but with all option names and 
%       option values removed.
%   
%       ISSET is a boolean vector stating which options in ALL_OPTS were 
%       set by input arguments (true), and which were copied from 
%       DEFAULTS_OPTS (false). Values explicitly set to a value equal to 
%       the default are refered to as true. The order of the bits 
%       is the same as the fields in DEFAULT_OPTS structure.
%
%       When the input argument '--' is used, then all input arguments that 
%       follow will be returned as regular arguments.  This allows string 
%       input arguments that follow '--' to start with a dash.
%
%   [ALL_OPTS, ARGS] = getopts('-nodash', DEFAULT_OPTS, ARG1, ARG2, ...., ARGN)
%       option names in ARG1 to ARGN have no leading dash ('-') character.
%       '-nodash' should be specified before DEFAULT_OPTS.
%       Using 'n-dash' is only recommended when normal arguments do not
%       contain any strings.  Otherwise unexpected results may occur.  When,
%       for example, one of the options is named 'delay' and a normal
%       argument is expected as to be a directory name, then strange things
%       may happen when a user specifies 'delay' as directory name.
%
%   [ALL_OPTS, ARGS] = getopts('-ignorecase', DEFAULT_OPTS, ARG1, ARG2, ...., ARGN)
%       option names in ARG1 to ARGN are not case sensitive. They will also
%       match a field in DEFAULT_OPTS when case does not match. 
%       '-ignorecase' should be specified before DEFAULT_OPTS.
%
%   When both options '-ignorecase' and '-dash' are used, the order is not
%   relevant, the functional behaviour is similar to how options are
%   interpreted in e.g. the function PLOT.
%
%   Typically GETOPTS is used inside a function to parse option arguments.
%   As alternative, GETOPTIONS is available to retrieve the complete set of
%   options for functions that use a structure to define options. 
%
% Example:
%       function myfunc(varargin)
%           default_opts.width  = 80;
%           default_opts.delete = 'off';
%           default_opts.force  = false;
%           [all_opts, args] = getopts(default_opts, varargin{:})
%           ...
%           % 'all_opts' can be further processed without extra checking.
%
%       When myfunc is called with:
%           myfunc('abc', '-width', 100, [4 5], '-force', '-d', 'on');
%
%       then 'all_opts' and 'args' will contain:
%           all_opts =
%                width:  100
%               delete: 'on'
%                force:  1
%
%           args = { 'abc' [4 5] }
%
%       Function WHAT_CALLS shows another example of how GETOPTS can be used
%       to parse input arguments of a function.
%
% see also: getoptions, varargin

%% last modified:
%   $Date: 2013-01-15 10:40:12 +0100 (Tue, 15 Jan 2013) $
%   $Author: jtoorn $
%   $Rev: 12634 $

%% history:
%   2012-09-10  jtoorn      add output value containing which options were
%                           specified and which were default
%   2012-01-21  biggelar    generate warning when option value has different
%                           class than default value
%                           break option parsing at '--' argument
%                           allow logical arrays as option value
%                           empty option values are not ignored
%   2011-10-15  biggelar    rename to getopts
%                           add -nodash and -ignorecase option
%                           ignore empty option values
%   2011-10-11  biggelar    add check for default boolean option to be FALSE
%                           add extra comment
%   2011-10-11  biggelar    create

function [all_opts, args, isset] = getopts(varargin)

% no recursion is used to prevent that -nodash and -ignorecase will 
% interfere with with other arguments.

%% check '-nodash' and '-ignorecase' which can be in any particular order

% check for '-nodash'
nodash = false;
if numel(varargin)>=1 
    if ischar(varargin{1})
        if strncmpi(varargin{1}, '-nodash', numel(varargin{1}))
            nodash = true;
            varargin(1) = [];   % remove argument
        end
    end      
end
        
% check for '-ignorecase'
if numel(varargin)>=1
    % define function handles to string compare functions
    % default case sensitive
    STRCMP  = @strcmp;
    STRNCMP = @strncmp;
    if ischar(varargin{1})
        if strncmpi(varargin{1}, '-ignorecase', numel(varargin{1}))
            STRCMP  = @strcmpi;
            STRNCMP = @strncmpi;
            varargin(1) = [];   % remove argument
        end
    end
end

% check for '-nodash' again for the case that '-ignorecase' was specified as the first argument
if numel(varargin)>=1
    if ischar(varargin{1})
        if strncmpi(varargin{1}, '-nodash', numel(varargin{1}))
            nodash = true;
            varargin(1) = [];   % remove argument
        end
    end
end
            
%% get DEFAULT_OPTS       
if numel(varargin)>=1
    default_opts = varargin{1};
    varargin(1) = [];       % remove argument
else
    error('no DEFAULT_OPTS specified');        
end                

%% check inputs
if ~isstruct(default_opts)
    error('DEFAULT_OPTS must be a structure');
end

%% option names
fldnames = fieldnames(default_opts);    % cell array with names of all options
if nodash
    optnames = fldnames;
else    
    optnames = strcat('-',fldnames);    % cell array with option strings starting with a dash '-'
end

%% check that scalar logical options are all initialized to FALSE
% The name of a boolean option should reflect the behaviour when that 
% option occurs in the input arguments. So, the TRUE value indicates the
% optional behaviour. Therefor, the default value for the boolean option 
% should be FALSE.
% If one feels to desire TRUE as default, consider renaming the option
% to reflect the inverse behaviour.
for i=1:numel(fldnames)
    value = default_opts.(fldnames{i});
    if islogical(value) && isscalar(value) && value == true;
        error(['Default for a boolean option ''' fldnames{i} ''' should be FALSE.']);
    end
end    

%% initialize output
all_opts = default_opts;

%% Initialize logical array that indicates which arguments are option names and values.
% Initalially assume that all arguments are options
isopt = true(size(varargin));       

%% initialize logical array that indicates which of all_opts are set by input arguments
isset = false(size(fldnames));

%% check each input argument
i=1;
nargs = numel(varargin);
while i<=nargs
    argument = varargin{i};
    % argument must be a one dimensional string to match an option name
    if ischar(argument) && size(argument,1)==1
    
        % if argument equals '--', then stop parsing for other options
        if strcmp(argument,'--')
            isopt(i+1:end) = false;  % all other options are regular arguments
            break;                   % stop while loop
        end
        
        % check if argument exactly matches one of the option names
        M = feval(STRCMP, argument, optnames);    % feval is required for R13
        if sum(M)==0
            % when no match is found, try truncated match.
            % all characters of the argument should match
            M = feval(STRNCMP, argument, optnames, numel(argument));
            if sum(M)>1
                % when more than 1 match is found, so argument is ambiguous
                error(['ambiguous option: ' argument]);
            end
        end    

        if sum(M)==0
            % When no matching option is found, this argument is not an option but a normal string.
            isopt(i) = false;
            % potential ambiguity may exist when '-nodash' is specified.
            if nodash
                warning('getopts:string_ambiguity', ...
                        ['String argument ''' argument ''' may conflict with an option name.' char(10), ...
                         'Consider using option names with a leading dash (''-'')']);
            elseif argument(1)=='-'
                % when argument starts with '-' this is an unknown option
                warning('getopts:unknow_option', 'unknown option: %s', argument);
            end
        else
            % exactly 1 match is found
            match = fldnames{M};
            isset(M) = true;
            % check if match is a scalar logical option or an option with a value
            if islogical(default_opts.(match)) && isscalar(default_opts.(match))
                % match is a logical
                all_opts.(match) = true;
            else
                % match is an option with a value
                % fill all_opts field with the value of next argument
                if i<nargs
                    value = varargin{i+1};
                    i = i+1;    % next input argument
                    % if default value is not empty,
                    % generate a warning when the class of VALUE differs
                    % from the class of the default value.
                    if ~isempty(all_opts.(match)) && ~strcmp(class(value),class(all_opts.(match)))
                        warning('getopts:class_mismatch', ...
                                'value of option %s has different class (%s) than default option (%s).', ...
                                match, class(value), class(all_opts.(match)));
                    end         
                    all_opts.(match) = value;
                else
                    error('last option is missing a value')
                end         
            end
        end
    else
        % argument is not a string. So, it can not be an option name
        isopt(i) = false;
    end
        
    i = i+1;    % next input argument
end

%% keep regular arguments by remove option names and values from input arguments
args = varargin(~isopt);
