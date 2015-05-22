function [A,B,C,D] = update_state_matrices(quadcopter_state)
%usage: [A,B,C,D] = update_state_matrices([0 0 0 0 0 0 0 0])
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

roll = quadcopter_state(1);
pitch = quadcopter_state(2);
yaw = quadcopter_state(3);

% U = pwm1 pwm2 pwm3 pwm4
% quadcopter constants
d= 0.032;
c_t =  0.0153; %N per pwm(0-1)
c_a =  0.6726e-03;% N/m per pwm (0-1)


%rpm to torque matrix "mixing matrix". 
M = [   d*c_t d*c_t -d*c_t -d*c_t;
        -d*c_t d*c_t d*c_t -d*c_t;
        c_a -c_a c_a -c_a];
    
F = [zeros(2,4); c_t c_t c_t c_t];

I = [   0.5119e-6 0 0; 
        0 0.5160e-6 0;
        0 0 0.5897e-6];
mass = 0.022; 
        
%torque_3 = M * [omega_m1^2; omega_m2^2; omega_m3^2; omega_m4^2]; 

%alpha_3 = torque_3\I; % = M *inv(I)*[omega_m1; omega_m2; omega_m3; omega_m4];

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

%alpha_x_2 = R32(1,:)*M *inv(I)*[omega_m1; omega_m2; omega_m3; omega_m4];
%alpha_y_1 = R31(2,:)*M *inv(I)*[omega_m1; omega_m2; omega_m3; omega_m4];
%alpha_z_0 = R30(3,:)*M *inv(I)*[omega_m1; omega_m2; omega_m3; omega_m4];

%acceleration_z_3

A=[ 0 0 0 1 0 0 0 0;
    0 0 0 0 1 0 0 0;
    0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1;
    0 0 0 0 0 0 0 0];

    
B = [   0 0 0 0;
        0 0 0 0;
        0 0 0 0;
        R32(1,:)*inv(I)*M ;
        R31(2,:)*inv(I)*M ;
        R30(3,:)*inv(I)*M ;
        0 0 0 0;
        R30(3,:)*(1/mass)*F ];
C = eye(8);
D = zeros(8,4);
end