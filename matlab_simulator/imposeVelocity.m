function [ newConfiguration ] = imposeVelocity( configuration, velocity )
%IMPOSEVELOCITY Given a homogeneous matrix 'configuration' rapresenting the configuration
%of the robot, and another homogeneous matrix 'velocity' rapresenting an instantaneous
%velocity to be applied to the robot, return the homogeneous matrix 'newConfiguration'
% resulting by applying that velocity for an instant to the robot (simply
% do the product between matrices)
newConfiguration = configuration;

% Sum translation part:
newConfiguration(1:2,3) = newConfiguration(1:2,3) + velocity(1:2,3);

% Product of rotational part
newConfiguration(1:2,1:2) = velocity(1:2,1:2) * newConfiguration(1:2,1:2);

end

