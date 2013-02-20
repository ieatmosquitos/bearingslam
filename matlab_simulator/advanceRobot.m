function [ nextConfig, homogVelocity] = advanceRobot( prevConfig, advanceVelocity, angVelocity )
% ADVANCEROBOT Given a robot configuration rapresented by homogeneous matrix 'prevConfig', a
% real number 'advanceVelocity', and a real angle 'angVelocity', return the 
% configuration 'nextConfig' of the robot obtained by applying the 'advanceVelocity' along x axis of robot frame, and angular
% velocity with respect to z axis of the robot.
% In addiction, return the applied velocity expressed with a homogeneous
% matrix 'homogVelocity'.

% Linear
translation = prevConfig(1:2,1:2)*[advanceVelocity,0].'; % Direction on movement is along x axis, respect to robot frame
% Angular
homogVelocity = valuesToHomog(translation(1),translation(2),angVelocity);
nextConfig = imposeVelocity(prevConfig, homogVelocity);

end

