function [ out ] = homogZToValues( R )
%HOMOGZTOVALUES Given an homogeneous rotation matrix around z, return
% the angle 'theta' and the vector 'displacement' extracted from the matrix

displacement = R(1:2,3);
theta = atan2(R(2,1),R(1,1));

out = [displacement;theta];

end

