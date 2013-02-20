function [ distTot ] = homogDistance( HR1, HR2 )
%HOMOGDISTANCE Given two homogeneous matrix, return an integer > 0
% that gives a measures of distances between them.

% Gives the ration of how much angular distance weight more than linear
% distance
angVSlinRatio = 10;

h1=homogZToValues(HR1);
h2=homogZToValues(HR2);
displacement1 = h1(1:2,1);
displacement2 = h2(1:2,1);

theta1 = h1(3,1);
theta2 = h2(3,1);

d = abs(theta1-theta2);

if d < pi
    disTheta = d;
else
    disTheta = pi - abs(pi-d);
end

distDisplac = norm(displacement1-displacement2);

distTot = angVSlinRatio*disTheta + distDisplac;

end

