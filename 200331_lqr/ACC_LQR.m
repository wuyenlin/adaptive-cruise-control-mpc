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
T = T_total/Ts;
v0 = 15;

%% Discretize the system

At   = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt   = [0; 0; B_f];
sys1 = ss(At,Bt,C_f,0);
sys2 = c2d(sys1,Ts,'zoh');
A    = sys2.A;
B    = sys2.B;
Cplot = sys2.C;                         % use full states for plotting
C     = zeros(3,3);                      % Measured outputs
%% Data in standard form

%Definition of the LTI system
LTI.A     = A; 
LTI.B     = B;
LTI.C     = C;
dim.N     = 20;
dim.nx    = size(A,1);
dim.nu    = size(B,2);
dim.ny    = size(C,1);
N         = 20;
x0        = [0;0;15];
vh        = host_velocity(v0,T);             % Define the host velocity
ub_x      = [2; 2.5; 40];
lb_x      = [0; 0; 15];
u_lim     = [5;3];
C_ulim    = [1;-1];
y_ref_OTS = [4*ones(1,T);                                 % delta d 
             zeros(1,T);              % delta v
             zeros(1,T)];             % host velocity

xhat0 = zeros(1,3);

% Optimal Target Selection reference states and inputs
x_ref_OTS = zeros(3,T);
u_ref_OTS = zeros(1,T);

x = zeros(length(A(:,1)),T);        % state trajectory
yplot = zeros(length(A(:,1)),T);    % output to plot
xhat = zeros(length(A(:,1)),T);     % estimated trajectories 
xhaug = zeros(length(A(:,1)),T);  % augmented states (3) x

u = zeros(length(B(1,:)),T);        % control inputs
y = zeros(length(C(:,1)),T);        % measurements 
yhat = zeros(length(C(:,1)),T);     % estimated output

e = zeros(length(A(:,1)),T);        % observer error
t = zeros(1,T);                     % time vector

Vf = zeros(1,T);                    % terminal cost sequence
l = zeros(1,T);                     % stage cost sequence

x(:,1) = x0';

%% Tuning weights
Q = 10*eye(size(A));            % state cost
R = 0.1*eye(length(B(1,:)));    % input cost
N = 20;
Qbar = kron(Q,eye(N));
Rbar = kron(R,eye(N));
[S,~,~] = dare(A,B,Q,R);
Sbar = S;
[P,Z,W] = predmodgen(LTI,dim);
H = (Z'*Qbar*Z + W'*Sbar*W);

%% Simulate system
disp('------------------------------------------------------------------');
disp('                Simulating Output MPC System');
disp('------------------------------------------------------------------');

for k = 1:1:T
    t(k) = (k-1)*Ts;
    % Optimal Target Selector
    Q_OTS = eye(3);
    R_OTS = eye(1);
    J_OTS = blkdiag(Q_OTS,R_OTS);
    
    A_OTS = [eye(3)-A     -B ;
             C         zeros(3,1)];
    b_OTS = [zeros(3,1)
             y_ref_OTS(:,k)]';

[xr_ur,~,exitflag] = quadprog(J_OTS,zeros(4,1),[],[],A_OTS,b_OTS,[],[],[]);

x_ref_OTS(:,k) = xr_ur(1:3);
u_ref_OTS(:,k) = xr_ur(end);

% determine reference states based on reference input r
% x_ref = B_ref*r(:,k);
x0_est = xhaug(1:3,k) - x_ref_OTS(:,k);

% compute control action
    cvx_begin quiet
        variable u_N(N)
        minimize ( (1/2)*quad_form(u_N,H))
        u_N >= -3;
        u_N <=  5;
    cvx_end
    u(k) = u_N(1); % MPC control action
% apply control action on real system
x(:,k+1)   = A*x(:,k) + B*u(:,k);
y(:,k)     = C*x(:,k);
yplot(:,k) = Cplot*x(:,k);

% stability analysis
Q = 10*eye(3);
R = 0.1*eye(1);

[X,eigvals,K] = dare(A,B,Q,R);
Vf(k) = 0.5*x(:,k)'*X*x(:,k);
l(k) = 0.5*x(:,k)'*Q*x(:,k);
end
% states_trajectory
states_trajectory = yplot';
saved_data.t = t;
saved_data.x = yplot;
saved_data.u = u;
%% 
figure(1);
clf;
hold on;
plot(t,xhaug(:,1:end-1));
%%
figure(2);
clf;
% plot(t,xhat(:,1:end-1));
hold on;
plot(t,x_ref_OTS(:,1:end));