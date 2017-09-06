function fbt_test_filter

% test that filters work OK on empty measurement struct
m            = fbt_meas;
fbt_filter_time(m, -1, 1);
assert(m.n == 0);
fbt_filter_select(m, 'cam', 1);
assert(m.n == 0);
fbt_filter_columns(m, {'az'});
assert(m.n == 0);

% test fbt_filter_select (which performs robot/cam filter)
m            = fbt_meas(10);
col          = fbt_col_idx('id', m);
m.data(:, col) = [1 1 1 2 2 2 3 3 4 4];
m.data(:, end) = 1:10;
m            = fbt_filter_select(m, 'id', [2 4]);
assertVectorsAlmostEqual(m.data(:, end), [4 5 6 9 10]');

% test fbt_filter_time
m            = fbt_meas(10);
col          = fbt_col_idx('t', m);
m.data(:, col) = 1:10;
m.data(:, end) = 11:20;
m            = fbt_filter_time(m, 3, 8);
assertVectorsAlmostEqual(m.data(:, col), [3:8]');
assertVectorsAlmostEqual(m.data(:, end), [13:18]');
assert(m.n == 6);

% test fbt_filter_columns
m            = fbt_meas(1);
m.data       = [1:11];
m            = fbt_filter_columns(m, {'t', 'conf', 'az'});
assert(isequal(m.columns, {'t', 'conf', 'az'}));
assertVectorsAlmostEqual(m.data, [3 11 8]);

1;
% TODO filter on 3d position? (x,y,z) or (bx,by,bz)?
