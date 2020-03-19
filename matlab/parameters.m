T_eng = 0.46;
T_brk = 0.193;
T_s = 0.05;
T_hw = 1.3;
K_brk = 0.979;

Ac_1 = [0 1 -T_hw; 0 0 -1; 0 0 -1/T_eng];
[V_1, D_1] = eig(Ac_1);
Ac_2 = [0 1 -T_hw; 0 0 -1; 0 0 -1/T_brk];
[V_2, D_2] = eig(Ac_2);