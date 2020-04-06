function plot_mpc(u,xr)

subplot(411);
stairs(u,'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('u');
title('Desired Acceleration Input u');
ylim('auto');
hold on;

subplot(413);
stairs(xr(1,1:200),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltad [m]');
title('State \deltad');
ylim('auto');
hold on;

subplot(412);
stairs(xr(2,1:200),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('\deltav [m/s]');
title('State \deltav');
ylim('auto');
hold on;

subplot(414);
stairs(xr(3,1)*ones(1,size(u,2)),'LineWidth',1.5);
grid on;
xlabel('time [seconds]');
ylabel('v_h [m/s]');
title('State v_h');
ylim('auto');
hold on;


end
