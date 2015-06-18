%% simulate
multirotorSimulator();

close all
%% plot roll

figure(2)
hold on
plot(simulationDatalog(1,:),simulationDatalog(2,:), 'r')%roll
plot(simulationDatalog(1,:),simulationDatalog(12,:), 'b')%aileron
title('Step response Roll')
legend('Actual Roll','Reference Roll')
xlabel('Time [s]')
ylabel('Roll [rad]')
hold off

%% plot pitch

figure(3)
hold on
plot(simulationDatalog(1,:),simulationDatalog(3,:), 'r')
plot(simulationDatalog(1,:),simulationDatalog(11,:), 'b')
title('Step response Pitch')
legend('Actual Pitch','Reference Pitch')
xlabel('Time [s]')
ylabel('Pitch [rad]')
hold off

%% plot yaw

figure(4)
hold on
plot(simulationDatalog(1,:),simulationDatalog(4,:), 'r')
plot(simulationDatalog(1,:),simulationDatalog(10,:), 'b')
title('Step response Yaw')
legend('Actual Yaw','Reference Yaw')
xlabel('Time [s]')
ylabel('Yaw[rad/s]')
hold off