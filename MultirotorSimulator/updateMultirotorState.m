function [ mr ] = updateMultirotorState( mr, pwms ,dt )
g = 9.81; % gravity
%the workflow here is to update the chain of states that depend on
%eachother in the reverse order so that we get the accurate behaviour as is
%expected. why: assume x(n+1) = A*x(n) + B*u(n)
% as you can see, if some of the states are integrated by the A matrice, it
% should integrate the old x and not the new one
% however the new input is used to change states by the inpus, in this case
% that is the propellers rpm

% % x,y,z
% mr.X = mr.X + dt*mr.VelX;
% mr.Y = mr.Y + dt*mr.VelY;
% mr.Z = mr.Z + dt*mr.VelZ;

%update dropdrives, these should be in the end if it doesn't
%instantaneously change rpm
for i = 1:length(mr.PropDrives)
    %disp(pwms(i));
    mr.PropDrives(i)= updatePropDriveState( mr.PropDrives(i), dt, pwms(i),3.7 );
end

% find torques and forces acting on body
% fXYZ3= zeros(3,1);%fXYZ3 is the forces in x,y,z which is the body coordinates
tXYZ3= zeros(3,1);%tXYZ3 is the torques in x,y,z which is the body coordinates
for pd = mr.PropDrives
    %torque on body
    tXYZ3= tXYZ3 + [pd.Lift*pd.Y; -pd.Lift*pd.X; pd.Torque] ;
%     
%     %force on body
%     fXYZ3= fXYZ3 +[0; 0; pd.Lift];
end

%TODO add dampening to velocities and rotational velocities

alpha_3 = mr.Inertia\tXYZ3; % find angular acceleration of quadcopter in body coordinates



[dummy, alpha_3(3),alpha_3(2),alpha_3(1)]= readJoystick();
tXYZ3 = mr.Inertia*50*alpha_3;

% acceleration_3= (1/mr.Mass)*fXYZ3; %find acceleration of quadcopter in body coordinates

% %rotation matrix from coordinate system 3 to 2
% R32 = [ 1 0 0;
%         0 cos(mr.Roll) sin(mr.Roll);
%         0 -sin(mr.Roll) cos(mr.Roll)];
%     
% %rotation matrix from coordinate system 3 to 1
% R31 = [ cos(mr.Pitch) 0 -sin(mr.Pitch);
%         sin(mr.Roll)*sin(mr.Pitch) cos(mr.Roll) sin(mr.Roll)*cos(mr.Pitch);
%         cos(mr.Roll)*sin(mr.Pitch) -sin(mr.Roll) cos(mr.Roll)*cos(mr.Pitch)];
%     
% %rotation matrix from coordinate system 3 to 0
% R30 = [ cos(mr.Yaw)*cos(mr.Pitch) sin(mr.Yaw)*cos(mr.Pitch) -sin(mr.Pitch);
%         cos(mr.Yaw)*sin(mr.Pitch)*sin(mr.Roll)-sin(mr.Yaw)*cos(mr.Roll) sin(mr.Yaw)*sin(mr.Pitch)*sin(mr.Roll)+cos(mr.Yaw)*cos(mr.Roll) cos(mr.Pitch)*sin(mr.Roll);
%         cos(mr.Yaw)*sin(mr.Pitch)*cos(mr.Roll)+sin(mr.Yaw)*sin(mr.Roll) sin(mr.Yaw)*sin(mr.Pitch)*cos(mr.Roll)-cos(mr.Yaw)*sin(mr.Roll) cos(mr.Pitch)*cos(mr.Roll)];
% 
%     
    
% R01 = [ cos(mr.Yaw)    -sin(mr.Yaw)   0;
%         sin(mr.Yaw)    cos(mr.Yaw)    0;
%         0           0           1];
%     
% R12 = [ cos(mr.Pitch)  0           sin(mr.Pitch);
%         0           1           0;
%         -sin(mr.Pitch) 0           cos(mr.Pitch)];
%     
% R23 = [ 1           0           0;
%         0           cos(mr.Roll)   -sin(mr.Roll);
%         0           sin(mr.Roll)   cos(mr.Roll)];
% 

omegaDot = mr.Inertia\( tXYZ3 - cross(mr.Omega,(mr.Inertia*mr.Omega)));
mr.Omega = mr.Omega + dt*omegaDot;


T = [   1   0               -sin(mr.Pitch);
        0   cos(mr.Roll)    cos(mr.Pitch)*sin(mr.Roll);
        0   -sin(mr.Roll)   cos(mr.Pitch)*cos(mr.Roll)];
    
temp = T\mr.Omega;    

mr.Roll = mr.Roll + dt*temp(1);
mr.Pitch = mr.Pitch + dt*temp(2);
mr.Yaw = mr.Yaw + dt*temp(3);

% %second derivative of x,y,z
% acceleration_x_0 = R30(1,:) * acceleration_3;
% acceleration_y_0 = R30(2,:) * acceleration_3;
% acceleration_z_0 = R30(3,:) * acceleration_3 - g;

% %Derivative of x,y,z
% mr.VelX = mr.VelX + dt*acceleration_x_0;
% mr.VelY = mr.VelY + dt*acceleration_y_0;
% mr.VelZ = mr.VelY + dt*acceleration_z_0;



end

