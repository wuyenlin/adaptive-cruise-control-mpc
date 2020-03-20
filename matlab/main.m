%% "Infinite horizon cost to go as terminal penalty"
clear all
close all
clc

%% Model definition

% Define model, cost function, and bounds.
A = [2, 1; 0, 2];
B = [1, 0; 0, 1];
N = 3;

alpha = 1e-5;
Q = alpha*eye(2);
R = eye(2);

% Bounds.
xlb = [-5; -inf()];
xub = [5; inf()];
ulb = [-1; -1];
uub = [1; 1];

% Find LQR.

[K, P] = dlqr(A, B, Q, R);
K = -K; % Sign convention.

%% Compute X_f.

Xn = struct();
V = struct();
Z = struct();

[Xn.('lqr'), V.('lqr'), Z.('lqr')] = findXn(A, B, K, N, xlb, xub, ulb, uub, 'lqr');

%% Simulate MPC from the initial starting point.

model = struct('A', A, 'B', B, 'N', N);
constraint = Z.lqr;
penalty = struct('Q', Q, 'R', R, 'P', P);
terminal = Xn.lqr{1}; % LQR terminal set.

Nsim = 25;
xsim = zeros(2, Nsim + 1);
xsim(:,1) = [0.5; 0.5]; % Initial condition.
usim = zeros(2, Nsim);
mpcmats = []; % Calculated first time and then reused.
for t = 1:Nsim
    model.x0 = xsim(:,t);
    [xk, uk, ~, status, mpcmats] = linearmpc(model, constraint, penalty, ...
                                             terminal, mpcmats);
    usim(:,t) = uk(:,1);
    xsim(:,t + 1) = xk(:,2);
end

figure,
plot(xsim(1,:), xsim(2,:), 'LineWidth', 2), grid on;

figure,
plot(0:Nsim-1, usim, 'LineWidth', 2), grid on;

%% Compute X_N by adding the terminal constraint.
[Xn.('termeq'), V.('termeq'), Z.('termeq')] = findXn(A, B, K, N, xlb, xub, ulb, uub, 'termeq');


%% 
% Now check and see where the finite-horizon control law is the same as the
% infinite-horizon control law.

% Let define a grid
[x1, x2] = meshgrid(linspace(-1.86, 1.86, 50), linspace(-0.97, 0.97, 50));
x0 = [x1(:), x2(:)]';
terminal = Xn.lqr{1};
X3 = Xn.lqr{end}; % Feasible set.
okaypts = all(bsxfun(@le, X3.A*x0, X3.b));
x0 = x0(:,okaypts); % Get rid of guys that are not feasible to save time.

Npts = size(x0, 2);
ininterior = all(bsxfun(@le, terminal.A*x0, terminal.b)); % See if start in Xf.
mpcmats = [];
for i = 1:Npts
    if ~ininterior(i)
        % Need to check with MPC problem.
        model.x0 = x0(:,i);
        [xk, uk, ~, status, mpcmats] = linearmpc(model, constraint, penalty, ...
                                                 terminal, mpcmats);
        if status.optimal == 0
            % Check whether the ending state is in the interior (with tolerance).
            ininterior(i) = all(terminal.A*xk(:,end) < terminal.b - 1e-3);
        end
    end
end
x0interior = x0(:,ininterior);

% Make plots.
datasets = {x0interior', 'Control Law = Inf. Horizon'};

for i = 1:size(datasets, 1)
    figure();
    pts = datasets{i, 1};
    plot(V.lqr{end}(1,:), V.lqr{end}(2,:), '-k', pts(:,1), pts(:,2), 'xk');
    xlabel('x_1');
    ylabel('x_2');
    title(datasets{i, 2});
end
