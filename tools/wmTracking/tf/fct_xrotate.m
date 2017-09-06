function M = fct_xrotate(a)
% 4D transformation matrix for xrotate (re-implementing Matlabs makehgtform)

ca = cos(a);
sa = sin(a);
M = [1 0 0 0; 0 ca -sa 0; 0 sa ca 0; 0 0 0 1];

