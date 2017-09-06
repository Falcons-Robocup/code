function [xo, yo, zo] = fct_transform(xr, yr, phir, xi, yi, zi, transformer)
% convert coordinates from one coordinate system to another
% inputs may be arrays (n*1)
% use 4D matrix transformations (not yet fully vectorized)


% check vectors all have the same length
n      = numel(xi);
% disable for speed
% assert(all(size(xr) == [n 1]));
% assert(all(size(yr) == [n 1]));
% assert(all(size(phir) == [n 1]));
% assert(all(size(xi) == [n 1]));
% assert(all(size(yi) == [n 1]));
% assert(all(size(zi) == [n 1]));

% convert per sample (TODO: vectorize for speed)
for it = 1:n
    % position
    pos      = [xi(it); yi(it); zi(it); 1];
    % get transformation matrix
    M        = feval(transformer, xr(it), yr(it), phir(it));
    newpos   = M * pos;
    xo(it,1) = newpos(1,:);
    yo(it,1) = newpos(2,:);
    zo(it,1) = newpos(3,:);
end

% TODO: velocity? rotation only, no translation
