function [A,B,C,D] = update_state_matrices(mr)
%usage: [A,B,C,D] = update_state_matrices(createMultirotor())
% states = [ro pi ya omega_x omega_y omega_z x y z velX velY velZ]
%TODO model propeller-dynamics, and use pwm as input

%TODO model motion in XYZ, ponder about "coupling terms" aka. coreolis
%effect and centripetal effect 

%TODO prevent negative PWM, and PWM over 255

%TODO verify correct behaviour by using working PID controller on simulated
%model

%TODO dampening velocities and rotational velocities

%TODO model thrust more accurately ( higher degree polynomial?) and effects
%from speed through air

%TODO model a crash vs. landing? when height is going from positive to
%negative, is quadcopter moving smoothly?

roll = mr.Roll;
pitch = mr.Pitch;
yaw = mr.Yaw;

% U = pwm1 pwm2 pwm3 pwm4
% quadcopter constants
d= 0.032; % meter, own measurement
% c_t =  0.09;%9.8*0.0153; %N per pwm(0-1)
% c_a = 1.5e-03;% 0.6726e-03;% N/m per pwm (0-1)
c_t =  9.8*0.0153; %N per pwm(0-1) from bitcraze measurements
c_a = 0.6726e-03;% N/m per pwm (0-1) from simulation

%rpm to torque matrix "mixing matrix". 
M = [   -d*c_t   -d*c_t   d*c_t  d*c_t;
        -d*c_t  d*c_t   d*c_t   -d*c_t;
        -c_a     c_a    -c_a     c_a];
    
F = [zeros(2,4); c_t c_t c_t c_t];
        
%torque_3 = M * [omega_m1^2; omega_m2^2; omega_m3^2; omega_m4^2]; 

%alpha_3 = torque_3\mr.Inertia; % = M *inv(mr.Inertia)*[omega_m1; omega_m2; omega_m3; omega_m4];

%rotation matrix from coordinate system 3 to 2
R32 = [ 1 0 0;
        0 cos(roll) sin(roll);
        0 -sin(roll) cos(roll)];
    
%rotation matrix from coordinate system 3 to 1
R31 = [ cos(pitch) 0 -sin(pitch);
        sin(roll)*sin(pitch) cos(roll) sin(roll)*cos(pitch);
        cos(roll)*sin(pitch) -sin(roll) cos(roll)*cos(pitch)];
    
%rotation matrix from coordinate system 3 to 0
R30 = [ cos(yaw)*cos(pitch) sin(yaw)*cos(pitch) -sin(pitch);
        cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) cos(pitch)*sin(roll);
        cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll) cos(pitch)*cos(roll)];

%alpha_x_2 = R32(1,:)*M *inv(mr.Inertia)*[omega_m1; omega_m2; omega_m3; omega_m4];
%alpha_y_1 = R31(2,:)*M *inv(mr.Inertia)*[omega_m1; omega_m2; omega_m3; omega_m4];
%alpha_z_0 = R30(3,:)*M *inv(mr.Inertia)*[omega_m1; omega_m2; omega_m3; omega_m4];

%acceleration_z_3

% the T matrix convert omega3 to rate of change of roll pitch and yaw
T = [   1   0               -sin(mr.Pitch);
        0   cos(mr.Roll)    cos(mr.Pitch)*sin(mr.Roll);
        0   -sin(mr.Roll)   cos(mr.Pitch)*cos(mr.Roll)];

A=[ zeros(3) inv(T) zeros(3,2);
    zeros(3,8);
    zeros(1,7) 1;
    zeros(1,8)]

B = [   0 0 0 0;
        0 0 0 0;
        0 0 0 0;
        R32(1,:)*inv(mr.Inertia)*M ;
        R31(2,:)*inv(mr.Inertia)*M ;
        R30(3,:)*inv(mr.Inertia)*M ;
        0 0 0 0;
        R30(3,:)*(1/mr.Mass)*F ]
C = eye(8);
D = zeros(8,4);
end