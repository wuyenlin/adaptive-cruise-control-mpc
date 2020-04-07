function plot_mpc(u,xr,t)

subplot(411);
stairs(t,u,'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('u');
title('Desired Acceleration Input u');
ylim('auto');
hold on;

subplot(413);
stairs(t,xr(1,1:end-1),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltad [m]');
title('State \deltad');
ylim('auto');
hold on;

subplot(412);
stairs(t,xr(2,1:end-1),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltav [m/s]');
title('State \deltav');
ylim('auto');
hold on;

subplot(414);
%stairs(t,xr(3,1)*ones(1,size(u,2)),'LineWidth',1.5);
stairs(t,xr(3,1:end-1),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\textbf{$\dot{v}_h$} [m/s]','interpreter','latex');
title('\textbf{State $\dot{v}_h$}','interpreter','latex');
ylim('auto');
hold on;


end
