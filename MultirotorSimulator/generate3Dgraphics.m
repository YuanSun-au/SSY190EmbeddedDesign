function graphics = generate3Dgraphics(mr)

% black is rear, blue is front if X config

%mostly used https://github.com/dch33/Quad-Sim/blob/master/Quadcopter%20Dynamic%20Modeling%20and%20Simulation/Graphical%20User%20Interfaces/QuadAnim4.m

% Generate the geometry used to draw the quadcopter
% r = 0.025; d = 0.06; h = .015; %inches: rotor dia., quad motor distance from d = sqrt(2)*0.032;
% % center of mass, and rotor height above arms (entirely cosmetic)
% a = 0.025; b = 0.03; c = 0.015; % top , bottom, and sides lengths of hub (?)
r = .5; d = 1.25; h = .25; %inches: rotor dia., quad motor distance from 
% center of mass, and rotor height above arms (entirely cosmetic)
a = 1; b = 1; c = 0.2; % top , bottom, and sides lengths of hub (?)


% Construct rotor representations
N = [d  0 h].';% m1 rotor center [X Y Z]
E = [0 -d h].';% m4 rotor center
W = [0  d h].';% m2 rotor center
S = [-d 0 h].';% m3 rotor center
nr = circlePoints(N, r, 10); nr = [nr nr(:,1)]; % Rotor blade circles
er = circlePoints(E, r, 10); er = [er er(:,1)];
wr = circlePoints(W, r, 10); wr = [wr wr(:,1)];
sr = circlePoints(S, r, 10); sr = [sr sr(:,1)];
% Motors connecting to center of blade circles
mn = [d,d;
      0,0;
      h,0];
me = [0,0;
     -d,-d;
      h,0];
mw = [0,0;
      d,d;
      h,0];
ms = [-d,-d;
       0,0;
       h,0];
% Construct body plot points
bns = [ d, -d;
        0,  0;
        0,  0]; %For drawing the body "X" shape
bew = [ 0,  0;
        d, -d;
        0,  0];
% Body (HUB) Squares
top = [ a/2,   0,-a/2,   0;
          0, b/2,   0,-b/2;
        c/2, c/2, c/2, c/2];
bot = vertcat(top(1:2,:),-top(3,:)); % bot is same as top just below the center of mass
neb = [ a/2, a/2,   0,   0;
          0,   0, b/2, b/2;
        c/2,-c/2,-c/2, c/2]; % By the way... north east is actually north west from above since x is north and y is east :P
nwb = [ a/2, a/2,   0,   0;
          0,   0,-b/2,-b/2;
        c/2,-c/2,-c/2, c/2];
seb = -nwb;
swb = -neb;


% % Rotate body frame velocity vector
% U = ones(3,1);%A(:,7);
% V = zeros(3,1);%A(:,8);
% W = zeros(3,1);%A(:,9);
% Vi = zeros(3,3);
% MvMax= max(sqrt(U.^2+V.^2+W.^2)); 
% Vb = 3/MvMax*[U(1), V(1), W(1)]'; % Scale velocity
% Vi(1,:) = R*Vb; % Rotate velocity vector to inertial frame for plotting

% Support for X-configuration
if (strcmp(mr.Mixing, 'X'))
    Rz = [ sqrt(2)/2, sqrt(2)/2, 0;
          -sqrt(2)/2,sqrt(2)/2, 0;
                   0,          0, 1];
    nr = Rz*nr;
    er = Rz*er;
    wr = Rz*wr;
    sr = Rz*sr;
    mn = Rz*mn;
    me = Rz*me;
    mw = Rz*mw;
    ms = Rz*ms;
    bns = Rz*bns;
    bew = Rz*bew;
    top = Rz*top;
    bot = Rz*bot;
    neb = Rz*neb;
    nwb = Rz*nwb;
    swb = Rz*swb;
    seb = Rz*seb;
end

graphics = struct( 'Nr',nr,...
    'Er',er,...
    'Wr',wr,...
    'Sr',sr,...
    'mN',mn,...
    'mE',me,...
    'mW',mw,...
    'mS',ms,...
    'bNS',bns,...
    'bEW',bew,...
    'Top',top,...
    'Bot',bot,...
    'NEB',neb,...
    'NWB',nwb,...
    'SWB',swb,...
    'SEB',seb);
end

function points = circlePoints(center, radius, numberOfPoints)
% Helper function for plotting points
% Inspired by "circle()" from Peter Corke's MATLAB Robotics Toolbox
c = center.'; % [x;y] location of center
r = radius;
n = numberOfPoints;
% compute points around the circumference
th = (0:n-1)'/n*2*pi; % angles coresponding to each point
x = r*cos(th) + c(1); % x part
y = r*sin(th) + c(2); % y part
points = [x,y].';
    if length(c) > 2
        z = ones(size(x))*c(3); % z part
        points = [x, y, z].';
    end
end