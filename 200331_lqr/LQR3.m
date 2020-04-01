%% ACC LQR MATLAB
clear all;
clc;

%% System Specifications
T_eng    = 0.460;
K_eng    = 0.732;
A_f      = -1/T_eng;
B_f      = -K_eng/T_eng;
T_hw     = 1.6;
Ts       = 0.05;
T_total  = 10; %simulation time
T        = T_total/Ts;
v0       = 15;

%% Create State-Space & Discretize the system

At    = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt    = [0; 0; B_f];
C_f   = eye(3);
D     = zeros(3,1);

sys1  = ss(At,Bt,C_f,D);
sys2  = c2d(sys1,Ts,'zoh');
A     = sys2.A;
B     = sys2.B;
C     = sys2.C;
D     = sys2.D;

%% LINEAR QUADRATIC REGULATOR

x0 = [0; 0; 15];

% reference sequence
r = [ 0*ones(1,(T+1));
      0*ones(1,(T+1));
      0*ones(1,(T+1));
      0*ones(1,(T+1))];
  
Q = transpose(C)*C;
R = 1;
N = 0;
[K,S,e] = dlqr(A,B,Q,R,N); 

B_ref = zeros(3,4);
B_ref(1,1) = 1;
B_ref(2,2) = 1;
B_ref(3,3) = 1;

%define closed loop system with LQR control law
sysd_cl_unnormalized = ss(A-B*K, B_ref, C, [], Ts);
%sysd_cl_unnormalized = ss(A, B, C, D, Ts);

%normalize closed-loop reference tracking gains 
dcgain_cl = dcgain(sysd_cl_unnormalized);
B_ref(1,1) = 1/dcgain_cl(1,1);
B_ref(2,2) = 1/dcgain_cl(2,2);
B_ref(3,3) = 1/dcgain_cl(3,3);

x = zeros(length(A(:,1)),T);
u = zeros(length(B(1,:)),T);
y = zeros(length(C(:,1)),T);
t = zeros(1,T);

x(:,1) = x0';

sat = @(s) min(max(s, -0.1), 0.1);

for k = 1:1:T
    t(k) = (k-1)*Ts;
    
    % compute control action
    u(:,k) = -K*x(:,k);  
    
    u(:,k) = sat(u(:,k));
    
    % apply control action
    x(:,k+1) = A*x(:,k) + B*u(:,k) + B_ref*r(:,k);
    y(:,k) = C*x(:,k);
end

% states_trajectory: Nx16 matrix of trajectory of 16 states
states_trajectory = y';

%% PLOT RESULTS
% plot 2D results
figure(98);
for i = 1:3
    subplot(3,1,i);
    stairs(t, states_trajectory(:,i));
    xlabel('Time [s]')
end  