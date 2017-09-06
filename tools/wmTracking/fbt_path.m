function fbt_path()
% add all subfolders to the path



basefolder = fileparts(mfilename('fullpath'));
subfolders = {'io', 'types', 'ops', 'gui', 'model', 'settings', 'sim', 'tf', 'util', 'tst'};
for ifolder = 1:numel(subfolders)
    addpath([basefolder filesep subfolders{ifolder}]);
end

warning('off', 'getopts:unknow_option');
