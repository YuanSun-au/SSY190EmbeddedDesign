function [ pd ]= updatePropDriveState( pd, dt, pwm, Voltage )
%Update the state of the motor and propeller combo given the current state and the input
%pd is the propDrive struct

if pwm > 1
    pwm = 1;
elseif pwm < 0
    pwm = 0;
end

if pd.Inertia == 0 %instant rpm change
    %pwm = 255*pwm;
    %pd.Lift = (0.001/4)*(0.409e-3*pwm^2 + 140-5e-3*pwm);%https://www.bitcraze.io/wp-content/uploads/2015/02/pwm_to_thrust.png
    
    rpm = 3.5992e+04*pwm^3   -6.4841e+04*pwm^2    +5.4086e+04*pwm    +0.0880e+04; %from propDriveTorqueCalculations
    pd.Omega = rpm *2*pi/60;
    
    pd.Torque = -4e-17*rpm^3 + 2.5e-12*rpm^2 -12e-09*rpm +1.7e-06;
    
    
else% propeller dynamics modelled
    % motor modell: U = RI + Kv*Rpm(in radians/s) ==> I=(U-Kv*Rpm)/R;
    rpm = pd.Omega * 60/(2*pi);
    propdrag = -4e-17*rpm^3 + 2.5e-12*rpm^2 -12e-09*rpm +1.7e-06;
    alpha = (pd.Torque-propdrag)/pd.Inertia;
    pd.Omega = pd.Omega + dt*alpha;
    
    Utot = pwm*Voltage;
    pd.I=(Utot-pd.Omega/pd.Kv)/pd.R; %R
    pd.Torque = pd.I*pd.Kt;
    
    
    
    rpm = pd.Omega * 60/(2*pi);
    propdrag = -4e-17*rpm^3 + 2.5e-12*rpm^2 -12e-09*rpm +1.7e-06;
    
    %pd.Lift = pd.Ct*(pd.Omega)^2;%TODO model thrust more accurately ( higher degree polynomial?) and effects 
end

pd.Lift = 2.7355e-11*rpm^2 - 5.2648e-08*rpm + 3.8542e-05;

%direction is taken as seen from above
if strcmp(pd.Direction, 'CCW')
    pd.Torque = - pd.Torque;
elseif strcmp(pd.Direction, 'CW')
    
else
    error('direction must be either CW or CCW');
end




end

