%% ACC MPC MATLAB Model Predictive Control SC42125
% Nikhil Hudrali Nagendra // Yen-Lin Wu
clc;
clear all;
%% Define system specifications (assuming the target is lagging behind the host vehicle)

T_eng     = 0.460;
K_eng     = 0.732;
A_f       = -1/T_eng;
B_f       = -K_eng/T_eng;
C_f       = eye(3);
T_hw      = 1.6;
Ts        = 0.05;
T_total   = 10;
T         = T_total/Ts;
v0        = 15;           % Initial target velocity
init_dist = 5;            % Intial distance between vehicles

%% Discretize the system

At    = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bt    = [0; 0; B_f];
sys1  = ss(At,Bt,C_f,0);
sys2  = c2d(sys1,Ts,'zoh');
A     = sys2.A;
B     = sys2.B;
C     = sys2.C;
% Host vehicle parameters
vh      = host_velocity(v0,T);               % Host velocity
sec_row = [init_dist*ones(1,length(vh))];
x0      = [sec_row;                     % Host intial distance error
           sec_row;                     % Host initial velocity error
           vh];                         % Host intial velocity           
u  = zeros(1,length(vh));               % Input acceleration or throttle matrix
  
xr = [zeros(1,length(vh));       
      zeros(1,length(vh));       
      zeros(1,length(vh));];
  
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
N      = dim.N;

w_idx = [0.1,1,10];
for w = w_idx
    Q = w.*[C']*C;        % weight on output
    R = 1;                % weight on input

    %% Prediction model and cost function
    [P,S] = predmodgen(LTI,dim);
    [H,h] = costgen(P,S,Q,R,dim);
%% MPC Problem

    umin    = -3*ones(1,N);
    umax    = 5*ones(1,N);
    xr(:,1) = x0(:,1);
    y(:,1)  = C*x0(:,1);
    for i = 1:T
        t(i) = (i-1)*Ts;
        f = h*xr(:,i);
        Aueq = C*x0(:,1:N);
        bueq = y(:,1);
        options = optimoptions('quadprog','Display','off');
        warning off;
        [ures,~,exitflag] = quadprog(H,f,Aueq,bueq,[],[],umin,umax,[],options);
        u(i) = ures(1);
        xr(:,i+1)   = A*xr(:,i) + B*u(i);
            y(:,i)  = C*xr(:,i+1);
            x(:,1)  = xr(:,i);
            for j = 1:N-1
                x(:,j+1)   = A*x(:,j)+B*ures(j);
                y_res(:,j) = C*x(:,j);
            end
    end
%% plot results
    plot_mpc(u,xr,t);
end

subplot(411);
legend({'W=0.1','W=1','W=10'});
axis([0 10 -2 6]);

subplot(412);
legend({'W=0.1','W=1','W=10'});

subplot(413);
legend({'W=0.1','W=1','W=10'});
axis([0 10 -10 10]);

subplot(414);
legend({'W=0.1','W=1','W=10'});
