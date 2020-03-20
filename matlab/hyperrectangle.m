function [A, b] = hyperrectangle(lb, ub)

% Returns halfspace representation of hyperrectangle with bounds lb and ub.

A = kron(eye(length(lb)), [1; -1]);
b = reshape([ub(:)'; -lb(:)'], 2*length(lb), 1);

% Any infinite or NaN bounds are ignored.
goodrows = ~isinf(b) & ~isnan(b);
A = A(goodrows,:);
b = b(goodrows);

end%function
