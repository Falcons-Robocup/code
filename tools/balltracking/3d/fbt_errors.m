function errors = fbt_errors(v)
% input argument: scale

if nargin == 0
    v       = 0; % by default return a clean struct
end

errors.az   = v * 0.05;  % relative error 3sigma
errors.el   = v * 0.05;  % relative error 3sigma
errors.r    = v * 0.35;  % relative error 3sigma
errors.loc  = v * 0.05;  % [m] 3sigma
errors.ts   = v * 0.005; % [s] 3sigma random delay per sample
errors.tr   = v * 0.015; % [s] 3sigma random systematic delay per robot (to be solved with NTP)
