function plot_mpc(u,xr,t)

subplot(221);
stairs(t,u,'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('u');
title('Desired Acceleration Input u');
ylim('auto');
hold on;

subplot(223);
stairs(t,xr(1,1:end-1),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltad [m]');
title('State \deltad');
ylim('auto');
hold on;

subplot(222);
stairs(t,xr(2,1:end-1),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltav [m/s]');
title('State \deltav');
ylim('auto');
hold on;

subplot(224);
stairs(t,xr(3,1)*ones(1,size(u,2)),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('v_h [m/s]');
title('State v_h');
ylim('auto');
hold on;


end
