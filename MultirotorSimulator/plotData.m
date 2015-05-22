function plotData( multirotor, time )
 % ----------- plot --------------
%     cla %clear plot
%     hold on
%     axis([-1.7 1.7 -1.7 1.7]) % size the window to a 10x10m square centerad around the real car position
%     axis equal
%     axis manual
%     
%     %plot stuff
%     % plot(DRPos(1:i,2),DRPos(1:i,3),'b');
%     
%     %represent robot as rectangle
%     rect1.Vertices=[0 -0.1;0.35 -0.1;0.35 0.1;0 0.1];
%     rect1.faces=[1 2 3 4];
%     
%     %rotation
%     rotationMatrix=[cos(theta) sin(theta);-sin(theta) cos(theta)];
%     rect1.Vertices=rect1.Vertices*rotationMatrix;
%     %translation
%     rect1.Vertices=rect1.Vertices+ ones(4,2)*[X 0; 0 Y];
%     patch(rect1,'Vertices',rect1.Vertices,'FaceColor','b');
%     
%     if(abs(turnRadius) < 10)
%         plot([X turnX],[Y turnY])
%     end
    
    drawnow

end

