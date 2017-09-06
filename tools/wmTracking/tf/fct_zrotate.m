function M = fct_zrotate(a)
% 4D transformation matrix for zrotate (re-implementing Matlabs makehgtform)

ca = cos(a);
sa = sin(a);
M = [ca -sa 0 0; sa ca 0 0; 0 0 1 0; 0 0 0 1];

