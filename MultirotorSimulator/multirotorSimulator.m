function []= multirotorSimulator()
%Quadcopter simulator

%TODO power monitoring, energy monitoring and temperature monitoring
clc
clf

%-------simulator settings-----------
maxSteps = 10000;
frequency= 500; %hz

dt =1/frequency;

iteration = 0;
collided = false;
%epsilon = 0.0001;

fps = 30; % how often to plot 
slowmo = 20; % percentage of full playback speed
showPlot = true;
% recordResults = false;

multirotor = createMultirotor();

% if (recordResults)
%     datalog = [];
% end

if(showPlot)
    pause on
end


while ((iteration < maxSteps) && (~collided))
    clc
    fprintf('\n Type ctrl-C to stop\n')
    time = iteration*dt;
    
    iteration

    [throttle, rudder, elevon, aileron]=readJoystick();
    if jst(5) ~= 0
        multirotor = createMultirotor(); %reset multirotor if button pushed
    end
    %pwms = controller(multirotor, throttle, rudder, elevon, aileron);
    pwms = [0 0 0 0];  
 
    [ multirotor ] = updateMultirotorState(multirotor, pwms ,dt );
    %multirotor
%     multirotor.PropDrives(1).Omega
%     multirotor.PropDrives(2).Omega
%     multirotor.PropDrives(3).Omega
%     multirotor.PropDrives(4).Omega
%     if (recordResults)
%         datalog = logData(datalog,time, multirotor);
%     end

%     collided = checkForCollisions(multirotor);
    
    %-------graphical representation of simulation-----------
    if ((mod(iteration,(frequency*slowmo/(100*fps)))==0) && showPlot)
        show3D(multirotor);
        plotData(multirotor, time)
        pause(dt*100/slowmo);
    end
    
    iteration = iteration + 1;
end

if(showPlot)
    pause off
end

end %robotSimulator
