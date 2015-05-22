clear
clf
dt = 0.01;
time = 0:dt:0.03;
pwm = linspace(0,0.9375,16);
voltage = [4.01, 3.98, 3.95, 3.92, 3.88, 3.84, 3.80, 3.76, 3.71, 3.67, 3.65, 3.62, 3.56, 3.48, 3.40, 3.30];

for i = 1:length(pwm)
    pd = createPropDrive(-0.032,0.032,'CW');
    for j = 1:length(time);
        %[throttle, rudder, elevon, aileron]=readJoystick();
        pd = updatePropDriveState( pd, dt, pwm(i), voltage(i) );
    end
    omega(i)= pd.Omega;
    I(i) = pd.I;
    Lift(i) = pd.Lift;
    torque(i) = pd.Torque;
end

subplot(1,3,1);
plot(pwm,(60/(2*pi))*omega, '-o')
hold on
rpmMeasured = [0, 4485, 7570,9374,10885,12277,13522,14691,15924,17174,18179,19397,20539,21692,22598,23882];
plot(pwm,rpmMeasured, 'ro-')
title('rpm');

subplot(1,3,2);
plot(pwm,torque,'-o');
title ('torque');
% plot(pwm,I, '-o')
% hold on
% Imeasured = 0.25*(-0.24*ones(1,16) +[0.24, 0.37, 0.56, 0.75, 0.94, 1.15, 1.37, 1.59, 1.83, 2.11, 2.39, 2.71, 3.06, 3.46, 3.88, 4.44]);
% plot(pwm,Imeasured, 'ro-')
% title('current');

subplot(1,3,3);
plot(pwm,Lift, '-o')
hold on
liftMeasured = 0.001*0.25*[0, 1.6, 4.8, 7.9, 10.9, 13.9, 17.3, 21.0, 24.4, 28.6, 32.8, 37.3, 41.7, 46.0, 51.9, 57.9];
plot(pwm,liftMeasured, 'ro-')
title('lift');