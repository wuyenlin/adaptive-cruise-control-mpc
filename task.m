%% p.6
T_eng = 0.46;
T_brk = 0.193;
T_s = 0.05;
T_hw = 1.3;


if u >= a_thr
    A_f = -1/T_eng;
    B_f = K_eng/T_eng;
else
    A_f = -1/T_brk;
    B_f = K_brk/T_brk;
end
C_f = 1;

Ac = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
Bc = [0; 0; B_f];
