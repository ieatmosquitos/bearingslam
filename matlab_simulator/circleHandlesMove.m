function circleHandlesMove( handles, center,radius,nPoints)
% Given a set of handles of a circle, translate them of the given vector

    angleDist = (2*pi)/nPoints;
    angle = 0;
    
    for i=1:nPoints
        set(handles{i},'xdata', center(1) + radius * cos(angle),'ydata',center(2) + radius * sin(angle));        
        angle = angle+angleDist;
    end
    


end

