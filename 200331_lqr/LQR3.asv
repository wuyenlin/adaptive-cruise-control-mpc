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

vh = host_velocity(v0, T_total);

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

x0 = [0;0;15];

% reference sequence
r = [ zeros(1,T);
      zeros(1,T);
      zeros(1,T);
      zeros(1,T)];
  
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

%normalize closed-loop reference tracking gains 
dcgain_cl = dcgain(sysd_cl_unnormalized);
B_ref(1,1) = 1/dcgain_cl(1,1);
B_ref(2,2) = 1/dcgain_cl(2,2);
B_ref(3,3) = 1/dcgain_cl(3,3);

x = zeros(length(A(:,1)),T);
u = zeros(length(B(1,:)),T);
y = zeros(length(C(:,1)),T);
t = zeros(1,T);

x(:,1) = x0(:,1);

for k = 1:1:T
    t(k) = (k-1)*Ts;
    % compute control action
    u(:,k) = -K*x(:,k);  
       
    % apply control action
    x(:,k+1) = A*x(:,k) + B*u(:,k) + B_ref*r(:,k);
    y(:,k)   = C*x(:,k);
end

states_trajectory = y';

%% PLOT RESULTS
% plot 2D results
figure(99);

subplot(3,1,1);
stairs(t, states_trajectory(:,1));
xlabel('Time [s]')
ylabel("");

subplot(3,1,2);
stairs(t, states_trajectory(:,2));
xlabel('Time [s]')
ylabel();

subplot(3,1,3);
stairs(t, v0*ones(1,T));
xlabel('Time [s]');
ylabel();