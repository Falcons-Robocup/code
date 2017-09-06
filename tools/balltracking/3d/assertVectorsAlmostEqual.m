function assertVectorsAlmostEqual(A, B, toltype, tol)
% re-implementation


if nargin < 4
    tol = sqrt(eps);
end
if nargin < 3
    toltype = 'relative';
end

if strcmp(toltype, 'relative')
    if ~(all(norm(A-B) <= tol*max(norm(A), norm(B)) + tol))
        disp(A);
        disp(B);
        error('assertion failed');
    end
else
    % absolute
    if ~(all(norm(A-B) <= tol))
        disp(A);
        disp(B);
        error('assertion failed');
    end
end

