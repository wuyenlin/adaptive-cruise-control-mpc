function [Xn, V, Z] = findXn(A, B, K, N, xlb, xub, ulb, uub, terminal)
% Calculates the various sets for the given system and parameters.
%
% Most inputs are self-explanatory. terminal should be either the string
% 'termeq' to use a terminal equality constraint, or 'lqr' to use the
% maximum LQR-invariant set.
%
% Output Xn is a cell array defining each of the \bbX_n sets. Xn{1} is the
% terminal set, Xn{2} is \bbX_1, etc. V is a cell array with each entry giving
% the extreme vertices of the corresponding Xn (as a 2 by N matrix). Z is a
% structure defining the feasible space.

% Define constraints for Z = {[x; u] | G x + H u + \psi <= 0}.
Nx = size(A, 1);
[Az, bz] = hyperrectangle([xlb; ulb], [xub; uub]);
Z = struct('G', Az(:,1:Nx), 'H', Az(:,(Nx + 1):end), 'psi', bz);

% Decide terminal constraint.
switch terminal
case 'termeq'
    % Equality constraint at the origin.
    Xf = [0; 0];
    
case 'lqr'
    % Build feasible region considering x \in X and Kx \in U.
    % Control input
    [A_U, b_U] = hyperrectangle(ulb, uub);
    A_lqr = A_U*K;
    b_lqr = b_U;
    
    % State input
    [A_X, b_X] = hyperrectangle(xlb, xub);
    Acon = [A_lqr; A_X];
    bcon = [b_lqr; b_X];

    % Use LQR-invariant set.
    Xf = struct();
    ApBK = A + B*K; % LQR evolution matrix.
    [Xf.A, Xf.b] = calcOinf(ApBK, Acon, bcon);
    [~, Xf.A, Xf.b] = removeredundantcon(Xf.A, Xf.b);

end

% Now do feasible sets computation.
figure();
hold('on');
colors = jet(N + 1);

Xn = cell(N + 1, 1);
Xn{1} = Xf;
names = cell(N + 1, 1);
names{1} = 'Xf';
V = cell(N + 1, 1);
for n = 1:(N + 1)
    % Plot current set.
    if n > 1
        names{n} = sprintf('X%d', n - 1);
    end
    
    plotargs = {'-o', 'color', colors(n,:)};
    
    V{n} = plotpoly(Xn{n}, plotargs{:});
    
    if n == (N + 1)
        break
    end

    % Compute next set. Also need to prune constraints.
    nextXn = computeX1(Z, A, B, Xn{n});
    [~, nextXn.A, nextXn.b] = removeredundantcon(nextXn.A, nextXn.b);
    Xn{n + 1} = nextXn;
end

legend(names{:});

end%function
