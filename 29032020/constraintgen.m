function [A,b]=constraintgen(dim)

A = kron(eye(5),[-1;1]);
b=[ones(dim.nu*dim.N,1);ones(dim.nu*dim.N,1)]*5;