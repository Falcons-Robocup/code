function fbt_test_readwrite



measurements = fbt_meas_read('testdata01.txt');
tmpfilename  = tempname;
fbt_meas_write(measurements, tmpfilename);
ok = sub_check_file_contents('testdata01.txt', tmpfilename);
delete(tmpfilename);
assert(ok == 1);


function ok = sub_check_file_contents(file1, file2)
    ok = 1;
    lines1 = strsplit(fileread(file1), '\n');
    lines2 = strsplit(fileread(file2), '\n');
    n = min(numel(lines1), numel(lines2));
    for iline = (n-1):-1:0
        if ~strcmp(lines1{end-iline}, lines2{end-iline})
            ok = 0;
            disp(lines1{end-iline});
            disp(lines2{end-iline});
            warning('lines differ');
            return
        end
    end
    