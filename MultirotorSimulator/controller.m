function [ pwms ] = controller( multirotor, throttle, rudder, elevon, aileron)

%TODO prevent negative PWM, and PWM over 1

%see figure 1 in modelling report 
m1 = throttle + rudder - elevon + aileron;
m2 = throttle - rudder + elevon + aileron;
m3 = throttle + rudder + elevon - aileron;
m4 = throttle - rudder - elevon - aileron;



pwms = 0.2*[m1 m2 m3 m4];
end

