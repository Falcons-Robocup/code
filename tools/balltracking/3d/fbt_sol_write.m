function fbt_sol_write(solutions, filename, writemode)
% write solutions to a file


% optionally open for append, but default overwrite
if nargin < 3
    writemode = 'w';
end

% open file
fid = fopen(filename, writemode);

% write
fprintf(fid, fbt_sol_2str(solutions));

% close file
fclose(fid);
