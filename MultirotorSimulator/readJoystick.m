function [throttle, rudder, elevon, aileron]=readJoystick()
% this probably only works on windows with the joystick vilse have.
% jst uses the .mexw32 and .mex64 files to read the joystick, depending on
% if the OS is 32 or 64 bit
joystick = jst;
throttle=-joystick(1);
elevon=joystick(4)*2-1; % left joystick have an interesting feature of ranging from 0 to 1 instead of -1 to 1 as the other axes on the joystick

rudder = -joystick(2);
aileron = joystick(3);
end