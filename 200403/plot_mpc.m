function plot_mpc(u,xr)

subplot(221);
stairs(u,'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('u');
title('Desired Acceleration Input u');
ylim('auto');
hold on;

subplot(223);
stairs(xr(1,:),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltad [m]');
title('State \deltad');
ylim('auto');
hold on;

subplot(222);
stairs(xr(2,:),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltav [m/s]');
title('State \deltav');
ylim('auto');
hold on;

subplot(224);
stairs(xr(3,1)*ones(1,size(u,2)),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('v_h [m/s]');
title('State v_h');
ylim('auto');
hold on;


end

