function M = fct_translate(x, y, z)
% 4D transformation matrix for translation (re-implementing Matlabs makehgtform)

M = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];

