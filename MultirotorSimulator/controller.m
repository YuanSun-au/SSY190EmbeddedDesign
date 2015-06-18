function [ pwms, dYaw ] = controller( mr,contr, throttle, rudder, elevon, aileron, sigmaMeasuredOmega)

T = [   1   0               -sin(mr.Pitch);
        0   cos(mr.Roll)    cos(mr.Pitch)*sin(mr.Roll);
        0   -sin(mr.Roll)   cos(mr.Pitch)*cos(mr.Roll)];
    
%temp_old = T\mr.Omega;  

% T*droyapi = omega -> dropiya = T\ omega, is implemented explicitly to be
% moved to the mcu
temp = (T(1,1)*T(2,2)*T(3,3)+T(1,2)*T(3,1)*T(2,3)+T(1,3)*T(2,1)*T(3,2))-(T(1,1)*T(2,3)*T(3,2)+T(1,2)*T(2,1)*T(3,3)+T(1,3)*T(2,2)*T(3,1));
dRoll = -((T(1,2)*T(3,3)*mr.Omega(2)+T(1,3)*T(2,2)*mr.Omega(3)+mr.Omega(1)*T(2,3)*T(3,2))-(T(1,2)*T(2,3)*mr.Omega(3)+T(1,3)*T(3,2)*mr.Omega(2)+mr.Omega(1)*T(2,2)*T(3,3)))/temp;
dPitch = -((T(1,1)*T(2,3)*mr.Omega(3)+T(1,3)*T(3,1)*mr.Omega(2)+mr.Omega(1)*T(2,1)*T(3,3))-(T(1,1)*T(3,3)*mr.Omega(2)+T(1,3)*T(2,1)*mr.Omega(3)+mr.Omega(1)*T(2,3)*T(3,1)))/temp;
dYaw = -((T(1,1)*T(3,2)*mr.Omega(2)+T(1,2)*T(2,1)*mr.Omega(3)+mr.Omega(1)*T(2,2)*T(3,1))-(T(1,1)*T(2,2)*mr.Omega(3)+T(1,2)*T(3,1)*mr.Omega(2)+mr.Omega(1)*T(2,1)*T(3,2)))/temp;

% state = [(mr.Roll-aileron), (mr.Pitch - elevon), mr.Yaw, dRoll,dPitch,(dYaw -10*rudder),mr.Pos(3)-mr.Pos(3),(mr.Vel(3) - 3*throttle)]';
% pwms = -contr.Kmat*state +0.52*ones(4,1);
state = [(mr.Roll-aileron), (mr.Pitch - elevon), 0, (dRoll + sigmaMeasuredOmega*randn), (dPitch + sigmaMeasuredOmega*randn),(dYaw -rudder + sigmaMeasuredOmega*randn),0,0]';

pwms = -contr.Kmat*state +(0.5*throttle+0.54)*ones(4,1); 

end

