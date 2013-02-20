function [ landmarks ] = genLandmarks( n, xMax ,yMax)
% Generate a set of 'n' bidimensional landmarks, uniformely distribuited
% in a space given by 'xMax','yMax'.
% Landmarks coordinates are contained in output nx2 matrix 'landmarks'.
% 'xMax' and 'yMax' must be positive real.
%
    xRand = rand(n,1)*xMax;
    yRand = rand(n,1)*yMax;   
    landmarks = [xRand,yRand];
    
end

 