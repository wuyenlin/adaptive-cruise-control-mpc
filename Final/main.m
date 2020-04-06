%% Model Predictive Control Project - SC42125
% Nikhil Hudrali Nagendra (5049628) // Yen-Lin Wu (4848489)
clc;
clear all;

disp('Model Predictive Control of a Basic Adaptive Cruise Control');
disp('Choose one of the options below');
disp('1. Basic ACC.');
disp('2. ACC MPC and ACC LQR Comparison.');
disp('3. MPC of ACC with varying prediction horizon N');
disp('4. MPC of ACC with varying Q');
disp('5. MPC of ACC with varying R');
disp('6. MPC of ACC with varying distance between cars');
pause(3);
choice = input('Choose now --> ');
switch choice
    case 1
        ACC_MPC_Final;
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
    case 2
        LQR_ACC;
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
    case 3
        ACC_MPC_varying_N;
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
    case 4
        ACC_MPC_varying_Q;
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
    case 5
        ACC_MPC_varying_R;
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
        case 6
        ACC_MPC_init_dist;
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
    otherwise 
        disp('Please choose a valid choice');
        disp('Do you want to try again?');
        opt = input('Enter your option [Y/N] --> ','s');
        if opt == 'Y'
            main;
        else
                disp('Thank You');
        end
end