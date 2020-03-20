function [AOinf, bOinf, tstar] = calcOinf(F, A, b)

% Calculates the maximum admissible set for x^+ = F*x subject to A*x <= b. Note
% that if F is unstable, this procedure will not work.

% Arguments and sizes.

Nx = size(F, 1);

Nc = size(A, 1);

b = b(:);

% Start the algorithm.
Ft = eye(Nx);
AOinf = zeros(0, Nx);
bOinf = zeros(0, 1);
tstar = inf();

for t = 0:100
    % Add constraints for new time point.
    AOinf = [AOinf; A*Ft];
    bOinf = [bOinf; b];

    % Recalculate objective.
    Ft = F*Ft;
    fobj = A*Ft;

    % Maximize each component, stopping early if any bounds are violated.
    okay = true();
    for i = 1:Nc
        [obj, feas] = solvelp_YALMIP(fobj(i,:), AOinf, bOinf);
        if ~feas || obj > b(i)
            okay = false(); % N isn't high enough Need to keep going.
            continue
        end
    end

    % If everything was feasible, then we're done.
    if okay
        tstar = t;
        break
    end
end

end%function


function [obj, feas] = solvelp_YALMIP(f, A, b)

    options = sdpsettings('verbose',0,'solver','quadprog');
    
    x_ = sdpvar(length(f),1);                % define optimization variable

    Constraint = [A*x_ <= b];                  %define constraints

    Objective = f*x_;  %define cost function

    diagnostic = optimize(Constraint,Objective, options);  %solve the problem
    x_=value(x_);                  %assign the solution to uopt
    obj = -f*x_;
    feas = (diagnostic.problem == 0);

end

