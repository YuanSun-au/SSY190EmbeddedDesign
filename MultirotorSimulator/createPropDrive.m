function motor = createPropDrive(x,y,direction)
% the propdrive struct contains all information of the parameters and state
% of one motor and propeller

motor = struct( 'R',1.6,...
                'L',0,... %inductance
                'Kv',1466,...
                'Kt',1/1466,...
                'I',0,... %current
                'Omega',0,... %rpm in radians/s
                'Inertia',0,...%2e-09,...%6.59e-010,... % from simple Solidworks model
                'Ca0',1.7e-06,... %torque constant from https://www.bitcraze.io/2015/02/measuring-propeller-rpm-part-3/
                'Ca1',-12e-09,... %torque constant
                'Ca2',2.5e-12,... %torque constant
                'Ca3',-4e-17,... %torque constant
                'Ct0',3.8542e-05,... %lift constant
                'Ct1',-5.2648e-08,... %lift constant
                'Ct2',2.7355e-11,... %lift constant
                'Lift',0,... %lift in N
                'Torque',0,...% torque in N/m
                'Direction',direction,...
                'X',x,... % position on quadcopter in x
                'Y',y); % position on quadcopter in y
end