% Plots for the quadcopter model
close all

for i = 1:3
    subplot(1,3,i)
    plot(quadcopter_state.signals.values(:,i))
    %title('Output angles')
    xlabel('time [s]')
    ylabel('angle [rad]')
    if i == 1
        title('Roll')
    elseif i == 2
        title('Pitch')
    elseif i == 3
        title('Yaw')
    else 
        
    end
    hold on
end

hold off