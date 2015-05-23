function trainTransition(west, east, WI, EI )
%WI is West input event, can be 0 = absent, 1=arrive, 2 = leave
%EI is East input event, can be 0 = absent, 1=arrive, 2 = leave
 %the states can be true (green) or false(red)
sigW = west;
 sigE = east;
 
 if(EI == 2)
     west = true;
 end
 if(WI == 2)
     east = true;
 end
 if (EI == 1)
     west = false;
 elseif(WI==1)
     east = false;
 end
 
 %%%%%%%%%%%%%%%%%%%%%%%%
 if(WI == 0)
     fprintf ('     ')
 elseif(WI == 1)
     fprintf ('Warr ')
 else
     fprintf ('Wlea ')
 end
 
 if(EI == 0)
     fprintf ('     ')
 elseif(EI == 1)
     fprintf ('Earr ')
 else
     fprintf ('Elea ')
 end
 %%%%%%%%%%%%%%%%%%%%%%%%%
 
 
 if(west)

     fprintf ('g')
 else
     fprintf ('r')
 end
 
 if(east)
     fprintf ('g')
 else
     fprintf ('r')
 end
 
 if(sigW)
     fprintf ('g')
 else
     fprintf ('r')
 end
 
 if(sigE)
     fprintf ('g')
 else
     fprintf ('r')
 end
      fprintf ('\r')
 
end

