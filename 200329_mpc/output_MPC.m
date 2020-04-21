%% Adaptive Cruise Control using MPC
%% INIT
clc
clear
addpath('Functions_MPC\');

%% DEFINE CONSTANTS
T_eng = 0.460;
K_eng = 0.732;
T_brk = 0.193;
K_brk = 0.979;
T_s   = 0.05;
T_hw  = 1.3;

%% DEFINE STATE SPACE SYSTEM
model = init_model(T_eng,K_eng,T_hw);
% check_controllability(model.A,model.B);

%% CHECK FOR STABILITY AND SET Xf
stability_analysis;
%% DISCRETIZE SYSTEM

% simulation time in seconds
h = T_s;

sys1 = c2d(model,h);
T = 3;

A = sys1.A;
B = sys1.B;
C = sys1.C;
Cplot = C;

%% MODEL PREDICTIVE CONTROL

% initial state
x0 = [0 0 0]';
xhat0 = zeros(1,3);

% desired reference (x,y,z,yaw)
r = [zeros(1,T/2) ones(1,T/2)];                   % yaw reference

% B_ref relates reference to states x_ref = B_ref*r
B_ref = zeros(3,1);
B_ref(3,1) = 1;

x     = zeros(length(A(:,1)),T);    % state trajectory
yplot = zeros(length(A(:,1)),T);% output to plot
xhat  = zeros(length(A(:,1)),T); % estimated trajectories 
u     = zeros(length(B(1,:)),T);    % control inputs
y     = zeros(length(C(:,1)),T);    % measurements 
yhat = zeros(length(C(:,1)),T); % estimated output
e = zeros(length(A(:,1)),T);    % observer error
t = zeros(1,T);                 % time vector

Vf = zeros(1,T);                % terminal cost sequence
l = zeros(1,T);                 % stage cost sequence

x(:,1) = x0';

% Define MPC Control Problem

% MPC cost function
%          N-1
% V(u_N) = Sum 1/2[ x(k)'Qx(k) + u(k)'Ru(k) ] + x(N)'Sx(N) 
%          k = 0

% tuning weights
Q = 10*eye(size(A));            % state cost
R = 0.1*eye(length(B(1,:)));    % input cost

% terminal cost = unconstrained optimal cost (Lec 5 pg 6)
[S,~,~] = dare(A,B,Q,R);        % terminal cost % OLD: S = 10*eye(size(A));

% prediction horizon
N = 20; 

Qbar = kron(Q,eye(N));
Rbar = kron(R,eye(N));
Sbar = S;

LTI.A = A;
LTI.B = B;
LTI.C = Cplot;

dim.N = N;
dim.nx = size(A,1);
dim.nu = size(B,2);
dim.ny = size(C,1);

[P,Z,W] = predmodgen(LTI,dim);
              
H = (Z'*Qbar*Z + Rbar + 2*W'*Sbar*W);
d = (x0'*P'*Qbar*Z + 2*x0'*(A^N)'*Sbar*W)';
 
%%

u_limit = [-3 5]';

Q_kf = 0.01*eye(3);
R_kf = 10*eye(1);

[~,Obs_eigvals,Obs_gain] = dare(A',C',R_kf,Q_kf);
Obs_gain = Obs_gain';

test = A-Obs_gain*C;

% measurement log
% 1
% 2
% 3
% 4
% 5
% 6
% 7
% 8
% 9
% 10
% 11 - OK
% 12 - OK
% 13 - OK
% 14 - OK
% 15
% 16 - 

for k = 1:1:T
    t(k) = (k-1)*h;
    
    % determine reference states based on reference input r
    x_ref = B_ref*r(:,k);
    x0_est = xhat(:,k) - x_ref;
    d = (x0_est'*P'*Qbar*Z + 2*x0_est'*(A^N)'*Sbar*W)';
    
    % compute control action
    cvx_begin quiet
        variable u_N(4*N)
        minimize ( (1/2)*quad_form(u_N,H) + d'*u_N )
        u_N >= -u_limit(1)*ones(4*N,1);
        u_N <=  u_limit(2)*ones(4*N,1);
    cvx_end
    
    u(:,k) = u_N(1:3); % MPC control action
    
    % apply control action on real system
    x(:,k+1) = A*x(:,k) + B*u(:,k); % + B_ref*r(:,k);
    y(:,k)   = C*x(:,k);
    
    yplot(:,k) = Cplot*x(:,k);
    
    % observer 
    yhat(:,k) = C*xhat(:,k);
    xhat(:,k+1) = A*xhat(:,k) + B*u(:,k) + Obs_gain*(y(:,k)-yhat(:,k));
    
    e(:,k) = x(:,k) - xhat(:,k);
    
    % stability analysis
    Q = 10*eye(3);
    R = 0.1*eye(1);
    
    [X,eigvals,K] = dare(A,B,Q,R);
    Vf(k) = 0.5*x(:,k)'*X*x(:,k);
    l(k) = 0.5*x(:,k)'*Q*x(:,k);
end

% states_trajectory: Nx16 matrix of output trajectory of 16 states
states_trajectory = yplot';