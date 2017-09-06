function fbt_meas_write(measurements, filename, writemode)
% write measurements to a file


% optionally open for append, but default overwrite
if nargin < 3
    writemode = 'w';
end

% open file
fid = fopen(filename, writemode);

% write
fprintf(fid, fbt_meas_2str(measurements));

% close file
fclose(fid);
