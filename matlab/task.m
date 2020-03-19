%% load parameters
clear all;
parameters;

%%
p = 20;
c = 1;

% define u and a_thr

if u >= a_thr
    A_c = [0 1 -T_hw; 0 0 -1; 0 0 -1/T_eng];
    B_c = [0; 0; K_eng/T_eng];
else
    A_c = [0 1 -T_hw; 0 0 -1; 0 0 -1/T_brk];
    B_c = [0; 0; K_brk/T_brk];
end

C_f = eye(3);

% Discretization
A_d = eye(3) + T_s * A_c;
B_d = T_s * B_c;
C_d = C_f;