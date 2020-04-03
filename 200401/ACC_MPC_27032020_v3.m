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
vh = host_velocity(v0,T_total);     % Host velocity

sec_row = [10, zeros(1,length(vh)-1)];
x0 = [sec_row;     % Host intial distance error
      sec_row;     % Host initial velocity error
      vh];                          % Host intial velocity

u  = zeros(1,length(vh));           % Input acceleration or throttle matrix
y  = C*x0;
xr = [zeros(1,length(vh));       
      zeros(1,length(vh));       
      zeros(1,length(vh));];
  
%x_ref = zeros(1,T_total);
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
Q = 10*eye(dim.ny);   % weight on output
R = 1;                % weight on input
%% Prediction model and cost function

[P,S] = predmodgen(LTI,dim);            %Generation of prediction model 
[H,h] = costgen(P,S,Q,R,dim);           %Writing cost function in quadratic form
%% To solve the quadratic MPC problem
lb = -3*ones(1,T);
ub = 5*ones(1,T);

%%%
f = 5*ones(20,1);
[x_ref,~,exitflag] = quadprog(H,f,[],[],[],[],lb,ub);
xr(:,1) = x0(:,1);
y_ref(:,1) = C*xr(:,1);
%%%

for j = 1:N-2
    xr(:,j+1) = A*xr(:,j) + B*x_ref(j);
    y_ref(:,j+1) = C*xr(:,j+1);
end

for i = 1:T
    %f = zeros(3,20);
    %[x_ref,~,exitflag] = quadprog(H,f,[],[],[],[],lb,ub);
    u_new(i)   = x_ref(1);
    x0(:,i+1)  = A*x0(:,i) + B*x_ref(1);
    y(:,i+1)   = C*x0(:,i+1);
    
%     for j = 1:N-1
%         if j == 1
%             xr(:,j) = x0(:,1);
%         else
%             xr(:,j) = A*xr(:,j-1) + B*x_ref(j-1);
%         end
%         y_ref(:,j) = C*xr(:,j);
%     end
end

figure(1);
subplot(2,2,1)
stairs(u_new,'LineWidth',1.5);
%stairs(x_ref(1),'LineWidth',1.5);
xlabel('time [seconds]')
grid on;
ylabel('u')
title('Desired Acceleration Input u')
ylim('auto')
hold on

subplot(2,2,4)
stairs([0 0 1]*x0,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('$vh$')
title('State $vh$')
ylim('auto')
hold on

subplot(2,2,3)
stairs([1 0 0]*x0,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('$delta d$')
title('State $delta d$')
ylim('auto')
hold on

subplot(2,2,2)
stairs([0 1 0]*x0,'LineWidth',1.5)
grid on;
xlabel('time [seconds]')
ylabel('$delta v$')
title('State $delta v$')
ylim('auto')

