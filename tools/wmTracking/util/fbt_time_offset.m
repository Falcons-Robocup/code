function t = fbt_time_offset
% to gain a few digits of numerical precision w.r.t. EPOCH
% if not using this, then we would have ~14us (=now*24*3600*eps) resolution, 
% which does not fit with printing in microseconds

t = datenum('2014-01-01') * 86400;
