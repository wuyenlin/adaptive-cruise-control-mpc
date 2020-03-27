function model = init_model(T_eng,K_eng,T_hw,T_brk,K_brk)
% Define system matrices.
if nargin <= 3
    A_f = -1/T_eng;
    B_f = -K_eng/T_eng;
    C_f = [1 0 0; 0 1 0; 0 0 1];
    At = [0 1 -T_hw; 0 0 -1; 0 0 A_f];
    Bt = [0; 0; B_f];
    model = ss(At,Bt,C_f,0);
else
    msg = 'Right now only 3 inputs';
    error(msg);
end