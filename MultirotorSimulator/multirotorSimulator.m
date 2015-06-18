function []= multirotorSimulator()
%Quadcopter simulator

%TODO power monitoring, energy monitoring and temperature monitoring
clc
clf

%-------simulator settings-----------
maxSteps = 2000;
frequency= 1000; %hz

dt =1/frequency;

iteration = 1;
collided = false;

fps = 60; % how often to plot 
slowmo = 100; % percentage of full playback speed
showPlot = true;
recordResults = true;

multirotor = createMultirotor();
controllerLQR= createController(multirotor);

if (recordResults)
    datalog = zeros(12,maxSteps);
end

if(showPlot)
    pause on
    graphics3D = generate3Dgraphics(multirotor);
end

%------- generate noise -----------
sigmaPwm = 0.05;
sigmaTorque = 3e-3;
sigmaForce = 0.5;
sigmaMeasuredOmega = 1e-2;

% we need to lowpass filter noise that represent outer forces & torques
if sigmaTorque > 0
    fc = 5;% filter cutoff frequency
    Wn = (2/frequency)*fc;
    filterLength = 2000;
    b = fir1(filterLength,Wn,'low',kaiser(filterLength+1,3));
    tempNoise = sigmaTorque*randn(3,maxSteps);
    noiseTorque = [filter(b,1,tempNoise(1,:));filter(b,1,tempNoise(2,:));filter(b,1,tempNoise(3,:))];
%     plot(linspace(0,maxSteps*dt,maxSteps),noiseTorque(1,:));
%     hold on
%     plot(linspace(0,maxSteps*dt,maxSteps),noiseTorque(2,:));
%     plot(linspace(0,maxSteps*dt,maxSteps),noiseTorque(3,:));
%     pause(60)
end

if sigmaForce > 0
    fc = 0.5;% filter cutoff frequency
    Wn = (2/frequency)*fc;
    filterLength = 4000;
    b = fir1(filterLength,Wn,'low',kaiser(filterLength+1,3));
    tempNoise = sigmaForce*randn(3,maxSteps);
    noiseForce = [filter(b,1,tempNoise(1,:));filter(b,1,tempNoise(2,:));filter(b,1,tempNoise(3,:))];
end

throttle = 0;
rudder = 0;
elevon = 0;
aileron = 0;
%------- simulation -----------
while ((iteration <= maxSteps) && (~collided))
    clc
    %fprintf('\n Type ctrl-C to stop\n')
    
    time = iteration *dt;

    %[throttle, rudder, elevon, aileron]=readJoystick();
    if iteration > 500
        rudder = pi;
    end
        
    if iteration > 1000
        rudder = 0;
    end
    
    
    if jst(5) ~= 0
        multirotor = createMultirotor(); %reset multirotor if button pushed
    end
    [pwms, dYaw]= controller(multirotor, controllerLQR, throttle, rudder, elevon, aileron, sigmaMeasuredOmega);
    
    pwms = pwms + sigmaPwm*randn(4,1); %adding noise
 
    [ multirotor ] = updateMultirotorState(multirotor, pwms , dt, noiseTorque(:,iteration), noiseForce(:,iteration));
    
    if (recordResults)
        datalog(:,iteration) = [time; multirotor.Roll; multirotor.Pitch; dYaw; pwms;throttle; rudder; elevon; aileron];
    end

%     collided = checkForCollisions(multirotor);
    
    %-------graphical representation of simulation-----------
    if ((mod(iteration,(frequency*slowmo/(100*fps)))==0) && showPlot)
        show3D(multirotor, graphics3D);
        plotData(multirotor, time)
        pause(dt*100/slowmo);
    end
    
    iteration = iteration + 1;
end

if(showPlot)
    pause off
end

if (recordResults)
    assignin('base', 'simulationDatalog', datalog);%store log in base workspace
end



end
