rpmMeasured = [0, 4485, 7570,9374,10885,12277,13522,14691,15924,17174,18179,19397,20539,21692,22598,23882];
liftMeasured = 0.001*0.25*[0, 1.6, 4.8, 7.9, 10.9, 13.9, 17.3, 21.0, 24.4, 28.6, 32.8, 37.3, 41.7, 46.0, 51.9, 57.9];
torque = 1.0e-03 *[-0.0070, -0.0088, 0.0215, 0.0665, 0.1164, 0.1657, 0.2119, 0.2545, 0.2938, 0.3311, 0.3680, 0.4063, 0.4479, 0.4948, 0.5486, 0.6106];

pwm = linspace(0,0.9375,16);

% p = polyfit(pwm,rpmMeasured,3);
% plot(pwm,rpmMeasured, 'ro-');
% hold on
% plot(pwm,polyval(p,pwm),'bo-');
% 
% %linearizing of pwm to torque
% p2 = polyfit(pwm,torque,1);
% plot(pwm,torque, 'ro-');
% hold on
% plot(pwm,polyval(p2,pwm),'bo-');

%linearizing of pwm to lift
p3 = polyfit(pwm,liftMeasured,1);
plot(pwm,liftMeasured, 'ro-');
hold on
plot(pwm,polyval(p3,pwm),'bo-');