function quality = fbt_solve_fit_quality(residuals)
% calculate fit quality based on residuals (part of confidence heuristic)


% tricky business: residuals vector contains quadruples (rx, ry, rz, r4)
% where r4 is an artificial coordinate and ry is very noisy (distance)
% so we consider only rx and rz
residuals_subset = residuals(1:2:end);
quality = sqrt(mean(residuals_subset.^2));
if (quality < 0.01) && (numel(residuals) > 25)
    % suspiciously good fit, simulation mode perhaps?
    1;
end
