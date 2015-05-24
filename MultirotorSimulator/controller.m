function [ pwms ] = controller( mr,contr, throttle, rudder, elevon, aileron)

T = [   1   0               -sin(mr.Pitch);
        0   cos(mr.Roll)    cos(mr.Pitch)*sin(mr.Roll);
        0   -sin(mr.Roll)   cos(mr.Pitch)*cos(mr.Roll)];
    
temp = T\mr.Omega;  


state = [(mr.Roll-aileron), (mr.Pitch - elevon), mr.Yaw, temp(1),temp(2),(temp(3) -10*rudder),(mr.Pos(3) - 2.5*throttle),mr.Vel(3)]';

pwms = -contr.Kmat*state +0.52*ones(4,1);

% %TODO prevent negative PWM, and PWM over 1
% 
% %see figure 1 in modelling report 
% m1 = throttle - rudder - elevon - aileron;
% m2 = throttle + rudder + elevon - aileron;
% m3 = throttle - rudder + elevon + aileron;
% m4 = throttle + rudder - elevon + aileron;
end

