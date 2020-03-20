clc;
clear all;

T_eng = 0.460;
K_eng = 0.732;
A_f = -1/T_eng;
B_f = -K_eng/T_eng;
C_f = [1 0 0; 0 1 0; 0 0 1];
T_hw = 1.6;
Ts = 0.05;

At = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt = [0; 0; B_f];
sys1 = ss(At,Bt,C_f,0);
sys2 = c2d(sys1,Ts,'zoh');

ctrbty = ctrb(sys2.A,sys2.B);
rankctrbty = rank(ctrbty);
obsrank = rank(obsv(sys2.A,sys2.C));
Q = diag([1 0 0]);
R = 1;
[P,K,L] = idare(sys2.A,sys2.B,Q,R);
Ad = sys2.A-K*sys2.C;
sys3 = ss(Ad,sys2.B,K*C_f,0);
sys3 = c2d(sys3,0.05,'zoh');

% check if system is observable
% and if P is SPD (symmetric and positive definite)
if rank(obsv(sys3)) && issymmetric(P)
    eigenvalue = eig(P);
    if all(eigenvalue > 0)
        fprintf("R=1,\nP= \n");
        disp(P);
    end
end
%% Find the terminal set Xf
%% Model definition

% Define model, cost function, and bounds.
A = sys3.A;
B = sys3.B;
N = 20;

% Bounds.
xlb = [0; 0; 15];
xub = [0; 2.5; 40];
ulb = -3;
uub = 5;

% Find LQR.

[K, P] = dlqr(A, B, Q, R);
K = -K; % Sign convention.

%% Compute X_f.

Xn = struct();
V = struct();
Z = struct();

[Xn.('lqr'), V.('lqr'), Z.('lqr')] = findXn(A, B, K, N, xlb, xub, ulb, uub, 'lqr');
