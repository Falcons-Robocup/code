function s = colprintf(data)
% Pretty Print struct variable in column style, recursively handling struct fields
% example:
%   >> colprintf(struct('b', [], 'd', false', 'a', 'firstline', 'c', struct('sub1', 'nested', 'sub2', pi)))
%      a      : firstline
%      b      : -empty- 
%      c.sub1 : nested
%      c.sub2 : 3.14159 
%      d      : false
% does not support cell elements, use prprintf instead

 
% initialize
options.format    = '%g ';
options.lwidth    = 'auto';
options.sort      = 1;
options.errcell   = 1;

% convert to string key/value pairs
assert(isstruct(data));
[k, v]            = sub_struct({}, {}, 0, '', options, data);

% glue columns
maxlen            = 0;
for irow = 1:numel(k)
    maxlen        = max(maxlen, numel(k{irow}));
end
format            = ['%-' num2str(maxlen) 's : %s\n'];
s                 = '';
for irow = 1:numel(k)
    s             = [s sprintf(format, k{irow}, v{irow})];
end


% helper functions

function [ko, vo] = sub_struct(ki, vi, depth, prefix, options, data)
    % initialize
    ko = ki;
    vo = vi;
    fn = fieldnames(data);  
    if options.sort
        fn = sort(fn);
    end
    % split the struct
    for ifield = 1:numel(fn)
        k = fn{ifield};
        pk = k;
        if numel(prefix)
            pk = [prefix '.' k];
        end
        v = getfield(data, k);
        if isstruct(v) % recurse
            [ko, vo] = sub_struct(ko, vo, 1+depth, pk, options, v);
        else
            % just convert the value to string, add line
            ko{end+1} = pk;
            vo{end+1} = sub_other(v, options);
        end
    end

    
function s = sub_other(v, options)
    % basic value converter
    if isempty(v)
        s = '-empty-';
    elseif ischar(v)
        s = v;
    elseif isnumeric(v)
        s = sprintf(options.format, v);
    elseif iscell(v)
        if options.errcell
            error('cell arrays are not supported');
        end
    elseif islogical(v)
        s = 'true';
        if v == false
            s = 'false';
        end
    else
        error('unsupported base type?!');
    end

    