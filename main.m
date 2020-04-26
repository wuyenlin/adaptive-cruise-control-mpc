%% Model Predictive Control Project - SC42125
% MPC for basic ACC System
% Nikhil Hudrali Nagendra (5049628) // Yen-Lin Wu (4848489)
clc;
clear all;
close all;
addpath('Code/');

disp('--- Model Predictive Control of a Basic Adaptive Cruise Control ---');
disp('Choose one of the options below');
disp('1. Basic ACC');
disp('2. ACC MPC and ACC LQR Comparison');
disp('3. MPC of ACC with varying prediction horizon N');
disp('4. MPC of ACC with varying Q');
disp('5. MPC of ACC with varying R');
disp('6. MPC of ACC with varying distance between cars');
disp('7. Stability analysis');
disp('0. Exit');
pause(1);
choice = input('Please choose --> ');
switch choice
    case 1
        close all;
        figure(1);
        ACC_MPC_Final;
        disp('Do you want to run another simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y'
            main;
        else
            disp('Goodbye!');
        end
    case 2
        close all;
        figure(2);
        LQR_ACC;
        disp('Do you want to run another simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y'
            main;
        else
            disp('Goodbye!');
        end
    case 3
        close all;
        figure(3);
        ACC_MPC_varying_N;
        disp('Do you want to run another simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y'
            main;
        else
            disp('Goodbye!');
        end
    case 4
        close all;
        figure(4);
        ACC_MPC_varying_Q;
        disp('Do you want to run another simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y'
            main;
        else
            disp('Goodbye!');
        end
    case 5
        close all;
        figure(5);
        ACC_MPC_varying_R;
        disp('Do you want to run another simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y'
            main;
        else
            disp('Goodbye!');
        end
    case 6
        close all;
        figure(6);
        ACC_MPC_init_dist;
        disp('Do you want to run another simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y' 
            main;
        else
            disp('Goodbye!');
        end
    case 7
        close all;
        figure(7);
        stability_analysis;
        disp('Do you want to run a1nother simulation?');
        opt = input('Enter your option [Y/any other key] --> ','s');
        if opt == 'Y'
            main;
        else
            disp('Goodbye!');
        end
    case 0
        disp('Goodbye!');
    otherwise 
        disp('Please input a valid number.');
        pause(2);
        main;
end
