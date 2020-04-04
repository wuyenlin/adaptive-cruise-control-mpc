%% ACC MPC MATLAB Model Predictive Control SC42125
% Nikhil Hudrali Nagendra // Yen-Lin Wu
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

At    = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt    = [0; 0; B_f];
sys1  = ss(At,Bt,C_f,0);
sys2  = c2d(sys1,Ts,'zoh');
A     = sys2.A;
B     = sys2.B;
C     = eye(3);

% Host vehicle parameters
vh      = host_velocity(v0,T);          % Host velocity
sec_row = [5, zeros(1,length(vh)-1)];
x0      = [sec_row;                     % Host intial distance error
           sec_row;                     % Host initial velocity error
           vh];                         % Host intial velocity           
u       = zeros(1,length(vh));          % Input acceleration or throttle matrix
  
xr    = [u;       
         u;       
         u];
  
x     = [zeros(1,T_total);zeros(1,T_total);zeros(1,T_total)];
s_lb  = [0;0;15];
s_ub  = [2;2.5;40]; 

%Definition of the LTI system
LTI.A = A;
LTI.B = B;
LTI.C = eye(3);

%Definition of system dimension
dim.nx = length(A);     % state dimension
dim.ny = length(B);     % output dimension
dim.nu = 1;             % input dimension
dim.N  = 20;            % horizon
N = dim.N;

%Definition of quadratic cost function
Q = eye(size(A,1));   % weight on output
R = 1;                % weight on input

[S,~,~] = dare(A,B,Q,R);
Qbar = kron(Q,eye(N));
Rbar = kron(R,eye(N));
Sbar = S;

%% Prediction model and cost function
[P,S] = predmodgen(LTI,dim);
[H,h] = costgen(P,S,Q,R,dim);

%% MPC Problem

umin = -3*ones(N,1);
umax = 5*ones(N,1);
xr(:,1) = x0(:,1);
y(:,1) = C*x0(:,1);
for i = 1:T
    f = S'*Qbar*P*xr(:,i);
    [ures,~,exitflag] = quadprog(H,f,[],[],[],[],umin,umax);
    u(i) = ures(1);
    xr(:,i+1) = A*xr(:,i) + B*u(i);
        y(:,i)  = C*xr(:,i+1);
        x(:,1) = xr(:,i);
        for j = 1:N-1
            x(:,j+1) = A*x(:,j) + B*ures(j);
            y_res(:,j) = C*x(:,j);
        end
end

%% plot results

figure(1);
subplot(2,2,1)
% stairs(u_new,'LineWidth',1.5);
stairs(u,'LineWidth',1.5);
xlabel('time [seconds]')
grid on;
ylabel('u')
title('Desired Acceleration Input u')
ylim('auto')
hold on

subplot(2,2,4)
stairs([0 0 1]*xr,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('v_h [m/s]')
title('State v_h')
ylim('auto')
hold on

subplot(2,2,3)
stairs([1 0 0]*xr,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('\deltad [m]')
title('State \deltad')
ylim('auto')
hold on

subplot(2,2,2)
stairs([0 1 0]*xr,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('\deltav [m/s]')
title('State \deltav')
ylim('auto')
            