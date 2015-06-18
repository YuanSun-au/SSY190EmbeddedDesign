function show3D( mr, gr )
clf
% usage: show3D( createMultirotor())

% ROTATION MATRIX --- ZYX ROTATION (R = Rib)
R = [cos(mr.Yaw)*cos(mr.Pitch) cos(mr.Yaw)*sin(mr.Pitch)*sin(mr.Roll)-sin(mr.Yaw)*cos(mr.Roll) cos(mr.Yaw)*sin(mr.Pitch)*cos(mr.Roll)+sin(mr.Yaw)*sin(mr.Roll);
       sin(mr.Yaw)*cos(mr.Pitch) sin(mr.Yaw)*sin(mr.Pitch)*sin(mr.Roll)+cos(mr.Yaw)*cos(mr.Roll) sin(mr.Yaw)*sin(mr.Pitch)*cos(mr.Roll)-cos(mr.Yaw)*sin(mr.Roll);
       -sin(mr.Pitch)         cos(mr.Pitch)*sin(mr.Roll)                            cos(mr.Pitch)*cos(mr.Roll)];


% Rotate body parts Via Initialized R
NrR = mr.Pos*ones(1,11) + R*gr.Nr;
ErR = mr.Pos*ones(1,11) + R*gr.Er;
WrR = mr.Pos*ones(1,11) + R*gr.Wr;
SrR = mr.Pos*ones(1,11) + R*gr.Sr;
mNr = mr.Pos*ones(1,2) + R*gr.mN;
mEr = mr.Pos*ones(1,2) + R*gr.mE;
mWr = mr.Pos*ones(1,2) + R*gr.mW;
mSr = mr.Pos*ones(1,2) + R*gr.mS;
bNSR = mr.Pos*ones(1,2) + R*gr.bNS;
bEWR = mr.Pos*ones(1,2) + R*gr.bEW;
TopR = mr.Pos*ones(1,4) + R*gr.Top;
BotR = mr.Pos*ones(1,4) + R*gr.Bot;
NEBR = mr.Pos*ones(1,4) + R*gr.NEB;
NWBR = mr.Pos*ones(1,4) + R*gr.NWB;
SWBR = mr.Pos*ones(1,4) + R*gr.SWB;
SEBR = mr.Pos*ones(1,4) + R*gr.SEB;

% Plot the box rotation and ang. velocity and inertial frame velocity
% vector
% axes(handles.axes1)
plot3(bNSR(1,:),bNSR(2,:),bNSR(3,:),'b','LineWidth',3) % Body Arm
hold on
plot3(bEWR(1,:),bEWR(2,:),bEWR(3,:),'b','LineWidth',3) % Body Arm
plot3(mNr(1,:),mNr(2,:),mNr(3,:),'k','LineWidth',4) % Motor
plot3(mEr(1,:),mEr(2,:),mEr(3,:),'k','LineWidth',4) % Motor
plot3(mWr(1,:),mWr(2,:),mWr(3,:),'k','LineWidth',4) % Motor
plot3(mSr(1,:),mSr(2,:),mSr(3,:),'k','LineWidth',4) % Motor
plot3(NrR(1,:),NrR(2,:),NrR(3,:),'g') % Motor 1 blades
plot3(ErR(1,:),ErR(2,:),ErR(3,:),'k') % Motor 4 blades
plot3(WrR(1,:),WrR(2,:),WrR(3,:),'k') % Motor 2 blades
plot3(SrR(1,:),SrR(2,:),SrR(3,:),'g') % Motor 3 blades
grey = [0.5 0.5 0.5];
top = fill3(TopR(1,:),TopR(2,:),TopR(3,:),'r'); alpha(top,0.8); % Top Surface
bot = fill3(BotR(1,:),BotR(2,:),BotR(3,:),'g'); alpha(bot,0.8); % Bottom Surface
ne  = fill3(NEBR(1,:),NEBR(2,:),NEBR(3,:),'c'); alpha(ne,0.8); % North East surface
nw  = fill3(NWBR(1,:),NWBR(2,:),NWBR(3,:),grey); alpha(nw,0.8); % North West surface
sw  = fill3(SWBR(1,:),SWBR(2,:),SWBR(3,:),'k'); alpha(sw,0.8); % South West surface
se  = fill3(SEBR(1,:),SEBR(2,:),SEBR(3,:),grey); alpha(se,0.8); % South East surface
% tempomega = 1e5*mr.Inertia*(inv(R)*mr.Omega)
% plot3( [0 tempomega(1)], [0 tempomega(2)],[0 tempomega(3)]);
axis square
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-5 5])
ylim([-5 5])
zlim([-5 5])
% view(handles.AZval,handles.ELval)
% grid on
% 
% omi = zeros(length(A),3); % Initialize omega inertiaF points array (L(A)x3)
% P = A(:,1); Q = A(:,2); Rw = A(:,3);
% MombMax = max(sqrt(P.^2+Q.^2+Rw.^2)); % Calculate max magnitude of omb
% omb = 3/MombMax*[P,Q,Rw].'; % Store and scale current omega bodyF
% omi(1,:) = R*omb(:,1); % Rotate omegab to inertiaF store in omegai array
% qp1 = quiver3(0,0,0,omi(1,1),omi(1,2),omi(1,3),'ro');
% qp2 = quiver3(0,0,0,Vi(1,1),Vi(1,2),Vi(1,3),'k');
% hold off

end
