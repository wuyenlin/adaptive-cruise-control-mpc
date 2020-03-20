function [nr, Anr, bnr, h, x0] = removeredundantcon(A, b, x0, tol, qhullopts)
% [nr, Anr, bnr, h, x0] = removeredundantcon(A, b, [x0], [tol], qhullopts)
%
% Finds the non-redundant constraints for the polyhedron Ax <= b. nr is a
% column vector with the non-redundant rows. If requested, Anr and bnr are the
% non-redundant parts of A and b.
%
% If x0 is supplied, it must be in the strict interior of A. Otherwise an error
% is thrown. Specifying a valid x0 will speed up the function.
%
% tol is used to decide how much on the interior we need to be. If not supplied,
% the default value is 1e-8*max(1, norm(b)/N). Note that this may be too large
% if b is poorly scaled.
%
% qhullopts is a string of options to pass to qhull. Defaults are described in
% the documentation for convhulln (see `help convhulln`).
%
% h is the output of convhulln, which actually describes the facets, and x0
% is a point on the interior of the polyhedron. Note that if the input
% polyhedron is unbounded, h may have zeros in some entries corresponding to the
% row of all zeros that needs to be added for the method to work.
%
% Note that this requires finding convex hull in N + 1 dimensions, where N
% is the number of columns of A. Thus, this will get very slow if A has a lot
% of columns.
narginchk(2, 5);

% Force b to column vector and check sizes.
b = b(:);
if isrow(b)
    b = b';
end
if size(A, 1) ~= length(b)
    error('A and b must have the same number of rows!');
end
if nargin() < 3
    x0 = [];
end
if nargin() < 4 || isempty(tol)
    tol = 1e-8*max(1, norm(b)/length(b));
elseif tol <= 0
    error('tol must be strictly positive!')
end
if nargin() < 5
    if size(A, 2) <= 4
        qhullopts = {'Qt'};
    else
        qhullopts = {'Qt', 'Qx'};
    end
end


% Save copies before things get messed up.
Anr = A;
bnr = b;

% First, get rid of any rows of A that are all zero.
Anorms = max(abs(A), [], 2);
badrows = (Anorms == 0);
if any(b(badrows) < 0)
    error('A has infeasible trivial rows.')
end
A(badrows, :) = [];
b(badrows) = [];
goodrows = [0; find(~badrows)]; % Add zero for all zero row that gets added.

% Need to find a point in the interior of the polyhedron.
if isempty(x0)
    if all(b > 0)
        % If b is strictly positive, we know the origin works.
        x0 = zeros(size(A, 2), 1);
    else
        error('Must supply an interior point!');
    end
else
    x0 = x0(:);
    if isrow(x0)
        x0 = x0';
    end
    if length(x0) ~= size(A,2)
        error('x0 must have as many entries as A has columns.')
    end
    if any(A*x0 >= b - tol)
        error('x0 is not in the strict interior of Ax <= b!')
    end
end

% Now, project the rows of P and find the convex hull.
btilde = b - A*x0;
if any(btilde <= 0)
    warning('Shifted b is not strictly positive. convhull will likely fail.')
end
Atilde = [zeros(1, size(A, 2)); bsxfun(@rdivide, A, btilde)];
h = convhulln(Atilde, qhullopts);
u = unique(h(:));

nr = goodrows(u);
if nr(1) == 0
    nr(1) = [];
end
h = goodrows(h);

% Finally, grab the appropriate rows for Anr and bnr.
Anr = Anr(nr, :);
bnr = bnr(nr);

end%function


