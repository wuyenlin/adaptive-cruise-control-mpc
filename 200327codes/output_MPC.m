%% Adaptive Cruise Control using MPC
%% Initialization
clc;
clear;
addpath('Functions_MPC\');

%% Define plant model

T_eng = 0.460;
K_eng = 0.732;
T_brk = 0.193;
K_brk = 0.979;
T_s   = 0.05;
T_hw  = 1.3;


A_f = -1/T_eng;
B_f = -K_eng/T_eng;
C_f = eye(3);

At = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt = [0; 0; B_f];
Ct = [1, 1, 1];
D = 0;
Ts = 0.05;

sys = ss(At, Bt, Ct, D, Ts);
x0 = [0; 2; 20]; %set initial state

%% Design uncontrained LQR
Qy = 1;
R = 0.01;
K_lqr = lqry(sys, Qy, R);

%% Run simulation

% figure(1);
% t_unconstrained = 0:0.05:10;
% u_unconstrained = zeros(size(t_unconstrained));
% Unconstrained_LQR = tf([1,1,1])*feedback(ss(At, Bt, eye(3),0,Ts),K_lqr);
% lsim(Unconstrained_LQR, '-', u_unconstrained, t_unconstrained, x0);
% hold on;
% grid on;
% axis([0 10 -4 4]);


%% LQR with constraints

x = x0;
t_constrained = 0:40;
uLQR = zeros(3,size(t_constrained, 2));
yLQR = zeros(1,size(t_constrained, 2));

for ct = t_constrained
    uLQR(ct+1) = -K_lqr*x;
    uLQR(ct+1) = max(-3,min(5,uLQR(ct+1)));
    x = At*x + Bt*uLQR(ct+1);
    yLQR(ct+1) = Ct*x;
end
figure(2);
subplot(2,1,1);
plot(t_constrained, uLQR);
xlabel('time');
ylabel('u');
grid on;

subplot(2,1,2);
plot(t_constrained, yLQR);
xlabel('time');
ylabel('y');
legend('Constrained LQR');
grid on;
