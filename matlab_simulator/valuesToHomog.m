function [ R ] = valuesToHomog( x,y,theta )
%VALUESTOHOMOG Given x,y return the 3x3 2d homogeneous
%matrix of rotation around z axis, with displacement vector x,y

R = [cos(theta),-sin(theta),x;
     sin(theta),cos(theta),y
     0  ,0  ,1];

end

