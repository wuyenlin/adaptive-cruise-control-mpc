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
%C_f   = eye(3);
C_f   = [0 0 1];
%D     = zeros(3,1);
D     = 0;

sys1  = ss(At,Bt,C_f,D);
sys2  = c2d(sys1,Ts,'zoh');
A     = sys2.A;
B     = sys2.B;
C     = sys2.C;
D     = sys2.D;

%% LINEAR QUADRATIC REGULATOR
p = 1; % weighing factor
Q = p*C'*C;
R = 1;
[K] = lqr(A,B,Q,R);

figure(25);
sys_cl = ss(A-B*K, B, C, D);
step(vh*sys_cl);