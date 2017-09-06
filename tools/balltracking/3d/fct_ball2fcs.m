function [bx, by, bz] = fct_ball2fcs(x, y, z, phi, az, el, r)
% convert ball measurements to FCS


% spherical to RCS
[bx, by, bz] = sph2cart(az + pi/2, el, r);
% convert to FCS
[bx, by, bz] = fct_rcs2fcs(x, y, phi, bx, by, bz);
% fix camera mounting height
bz = bz + z;
