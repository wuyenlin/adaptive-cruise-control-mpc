function plot_lqr(v0,states_trajectory,T)

subplot(4,1,1);
stairs(states_trajectory(:,1),'LineWidth',1.5);
xlabel('Time [s]')
ylabel("\deltad [m]");
title('State \deltad');
ylim('auto');
grid on;
hold on;

subplot(4,1,2);
stairs(states_trajectory(:,2),'LineWidth',1.5);
xlabel('Time [s]')
ylabel("\deltav [m/s]");
title('State \deltav');
ylim('auto');
grid on;
hold on;

subplot(4,1,3);
stairs(v0*ones(1,T),'LineWidth',1.5);
xlabel('Time [s]');
ylabel("host velocity v_h [m/s]");
title('State v_h');
ylim('auto');
grid on;
hold on;

end

