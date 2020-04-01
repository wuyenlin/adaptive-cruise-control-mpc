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
T_total  = 10;
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

%% Convert to transfer function & apply LQR 

[b,a] = ss2tf(A,B,C,D);

Q = transpose(C)*C;
R = 1;
N = 0;

[K,S,e] = lqr(A,B,Q,R,N);
sys4 = ss(A,B,C,D);

%% Plot the results

figure(1);
subplot(311);
step(sys4);

n  = length(K);
AA = A - B * K;
BB = zeros(3,length(K));
CC = C;
DD = D;

for i = 1:n
    BB(:,i) = B * K(i);
    sys4(:,i) = ss(AA,BB(:,i),CC,DD);
end

subplot(312);
step(sys4(:,2));

subplot(313);
step(sys4(:,3));
