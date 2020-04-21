%% ACC MPC MATLAB

clc;
clear all;
%% Define system specifications

T_eng = 0.460;
K_eng = 0.732;
A_f = -1/T_eng;
B_f = -K_eng/T_eng;
C_f = [1 0 0; 0 1 0; 0 0 1];
T_hw = 1.6;
Ts = 0.05;
T_total = 10;
N = T_total;
T = T_total/Ts;
v0 = 15;

%% Discreteize the system

At   = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt   = [0; 0; B_f];
sys1 = ss(At,Bt,C_f,0);
sys2 = c2d(sys1,Ts,'zoh');
A    = sys2.A;
B    = sys2.B;
C    = sys2.C;
%% Data in standard form

%Definition of the LTI system
LTI.A = A; 
LTI.B = B;
LTI.C = C;
x0    = [0;0;15];
vh    = host_velocity(v0,T);             % Define the host velocity
r     = [4*zeros(1,T); zeros(1,T); vh];  % Desired reference
B_ref = eye(3);

%% Constraints
ub_x   = [2; 2.5; 40];
lb_x   = [0; 0; 15];
u_lim  = [5;3];
C_ulim = [1;-1];
%Definition of system dimension
dim.nx = size(A,1);     % State dimension
dim.nu = size(B,2);     % Input dimension
dim.ny = size(C,1);     % Output dimension
dim.N  = 20;            % horizon

%Definition of quadratic cost function
Q     = 10*eye(size(A));            % state cost
R     = 1*eye(length(B(1,:)));      % input cost

x     = zeros(length(A(:,1)),T);    % state trajectory
yplot = zeros(length(A(:,1)),T);    % output to plot
xhat  = zeros(length(A(:,1)),T);    % estimated trajectories 
u     = zeros(length(B(1,:)),T);    % control inputs
y     = zeros(length(C(:,1)),T);    % measurements 
yhat  = zeros(length(C(:,1)),T);    % estimated output
e     = zeros(length(A(:,1)),T);    % observer error
t     = zeros(1,T);                 % time vector

Vf = zeros(1,T);                % terminal cost sequence
l = zeros(1,T);                 % stage cost sequence

x(:,1) = x0';

%% Prediction model and cost function

[P,S] = predmodgen(LTI,dim);           % Generation of prediction model

[H,h,const] = costgen(P,S,Q,R,dim,x0);   % Generation of the cost function

% Observer
Q_kf = 0.01*eye(3);
R_kf = 10*eye(1);

[~,Obs_eigvals,Obs_gain] = dare(A',C',R_kf,Q_kf);
Obs_gain = Obs_gain';

%%
for k = 1:1:T
    t(k) = (k-1)*Ts;
    
    % determine reference states based on reference input r
    x_ref = B_ref*r(:,k);
    x0_est = xhat(:,k) - x_ref;
    
    % compute control action
    cvx_begin quiet
        variables u_N(dim.nu*N)
        minimize ( 0.5*quad_form(u_N,H))
        subject to
        u_N <= 5
        u_N >= -3
    cvx_end
    
    u(k) = u_N(k); % MPC control action
    % apply control action on real system
    x(:,k+1) = A*x(:,k) + B*u(k); % + B_ref*r(:,k);
    y(:,k) = C*x(:,k);
    
    yplot(:,k) = C*x(:,k);
    
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
figure(1);
clf;
plot(t,xhat(1,1:end));
hold on;
plot(t,x(:,1:end));
plot(t,e(3,:));