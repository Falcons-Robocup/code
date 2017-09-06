function r = isfile(filename)

r = 0;
try
    fid = fopen(filename, 'r');
    assert(fid >= 0);
    r = 1;
end

