function s = fbt_sol_2str(solutions)
% convert solutions to string

% convert
s     = '';
for isol = 1:numel(solutions.n)
    if solutions.n(isol)
        % keep format consistent with ballStimulator.cpp
        si = sprintf('%16.6f  %2d  %5d  %4.2f  %7.3f  %7.3f  %7.3f  %7.3f  %7.3f  %7.3f\n', ...
            solutions.t(isol), solutions.n(isol), ... % timestamp, numBalls
            solutions.id(isol), solutions.conf(isol), ... % trackerId, confidence
            solutions.x(isol), solutions.y(isol), solutions.z(isol), ... % solved position
            solutions.vx(isol), solutions.vy(isol), solutions.vz(isol)); % solved speed 
    else
        si = sprintf('%16.6f  %2d  %5d  %4.2f  %7.3f  %7.3f  %7.3f  %7.3f  %7.3f  %7.3f\n', ...
            solutions.t(isol), solutions.n(isol), ...
            0, 0, 0, 0, 0, 0, 0, 0); % no balls, no detailed data
    end
    s = [s si];
end

% fix numerical zeros (which are annoying in diff-based testing)
s     = regexprep(s, ' -0.(0+\s)', '  0.$1');

