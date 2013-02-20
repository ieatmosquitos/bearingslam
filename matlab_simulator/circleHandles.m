function [ handles ] = circleHandles(center,radius,nPoints)
%CIRCLEHANDLES 

    angleDist = (2*pi)/nPoints;
    angle = 0;
    handles = cell(nPoints);
    
    for i=1:nPoints
        handles{i} = plot(center(1) + radius * cos(angle),center(2) + radius * sin(angle) ,'--mo','color','g');
        angle = angle+angleDist;
    end
end

