function s = fbt_time_float2str(t)

us = sprintf('%06d', round(rem(t, 1) * 1e6));
s = [strrep(datestr((t + fbt_time_offset) / 86400, 31), ' ', ',') '.' us];
