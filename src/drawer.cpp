#include "drawer.h"
#include "Eigen/Core"
#include <vector>

rdrawer::RobotDrawer::RobotDrawer(){
  _window_width = 1024;
  _window_height = 768;
  
  _window.Create(sf::VideoMode(_window_width,_window_height,32), "Robot Drawer");
  
  _camera_rect = sf::Rect<float>(0,0,_window_width, _window_height);
  _camera.SetFromRect(_camera_rect);
  
  _backgroundImage.LoadFromFile("assets/rd/wood-puzzle-floor.png");
  _backgroundSprite.SetImage(_backgroundImage); 
  
  _robotImage.LoadFromFile("assets/rd/roomba.png");
  _robotSprite.SetImage(_robotImage);
  _robotSprite.SetCenter(_robotSprite.GetSize().x/2, _robotSprite.GetSize().y/2);
  
  _traj_stepImage.LoadFromFile("assets/rd/trajectory_step.png");
  _traj_stepSprite.SetImage(_traj_stepImage);
  _traj_stepSprite.SetCenter(_traj_stepSprite.GetSize().x/2, _traj_stepSprite.GetSize().y/2);
  
  _landmarkImage.LoadFromFile("assets/rd/landmark.png");
  _landmarkSprite.SetImage(_landmarkImage);
  _landmarkSprite.SetCenter(_landmarkSprite.GetSize().x/2, _landmarkSprite.GetSize().y/2);
  
  _unconf_landmarkImage.LoadFromFile("assets/rd/unconfirmed_landmark.png");
  _unconf_landmarkSprite.SetImage(_unconf_landmarkImage);
  _unconf_landmarkSprite.SetCenter(_unconf_landmarkSprite.GetSize().x/2, _unconf_landmarkSprite.GetSize().y/2);
  
  _robot_pose.x = 0;
  _robot_pose.y = 0;
  _robot_pose.theta = 0;
}

rdrawer::RobotDrawer::~RobotDrawer(){}

void rdrawer::RobotDrawer::setRobotPose(RobotPose r){
  this->_robot_pose = r;
}

void rdrawer::RobotDrawer::addLandmark(rdrawer::Landmark * l){
  landmarks.push_back(l);
}

void rdrawer::RobotDrawer::addAndCreateLandmark(double x, double y){
  rdrawer::Landmark * l = new rdrawer::Landmark(x,y);
  landmarks.push_back(l);
}

void rdrawer::RobotDrawer::clearLandmarksList(){
  this->landmarks.clear();
}

void rdrawer::RobotDrawer::clearAndDeleteLandmarks(){
  for(unsigned int i=0; i<landmarks.size(); i++){
    delete (landmarks[i]);
  }
  landmarks.clear();
}

void rdrawer::RobotDrawer::addUnconfirmedLandmark(rdrawer::Landmark * l){
  unconfirmed_landmarks.push_back(l);
}

void rdrawer::RobotDrawer::addAndCreateUnconfirmedLandmark(double x, double y){
  rdrawer::Landmark * l = new rdrawer::Landmark(x,y);
  unconfirmed_landmarks.push_back(l);
}

void rdrawer::RobotDrawer::clearUnconfirmedLandmarksList(){
  this->unconfirmed_landmarks.clear();
}

void rdrawer::RobotDrawer::clearAndDeleteUnconfirmedLandmarks(){
  for(unsigned int i=0; i<unconfirmed_landmarks.size(); i++){
    delete (unconfirmed_landmarks[i]);
  }
  this->unconfirmed_landmarks.clear();
}

void rdrawer::RobotDrawer::addTrajectoryStep(double x, double y, double theta){
  Eigen::Vector3d v(x,y,theta);
  trajectory.push_back(v);
}

void rdrawer::RobotDrawer::clearTrajectory(){
  trajectory.clear();
}

void rdrawer::RobotDrawer::clearAll(){
  this->clearAndDeleteUnconfirmedLandmarks();
  this->clearAndDeleteLandmarks();
  this->clearTrajectory();
}

void rdrawer::RobotDrawer::draw(){
  _window.Clear();
  
  _window.SetView(this->_camera);
  
  // draw the background
  _window.Draw(_backgroundSprite);
  
  // draw the landmarks
  for(unsigned int i=0; i<landmarks.size(); i++){
    _landmarkSprite.SetPosition(landmarks[i]->x + _window_width/2, _window_height/2 - landmarks[i]->y);
    _window.Draw(_landmarkSprite);
  }
  
  // draw the unconfirmed landmarks
  for(unsigned int i=0; i<unconfirmed_landmarks.size(); i++){
    _unconf_landmarkSprite.SetPosition(unconfirmed_landmarks[i]->x + _window_width/2, _window_height/2 - unconfirmed_landmarks[i]->y);
    _window.Draw(_unconf_landmarkSprite);
  }
  
  // draw the trajectory
  for(unsigned int s=0; s<trajectory.size(); s++){
    _traj_stepSprite.SetPosition(trajectory[s][0] + _window_width/2, _window_height/2 - trajectory[s][1]);
    _traj_stepSprite.SetRotation(trajectory[s][2] * 180 / M_PI);
    _window.Draw(_traj_stepSprite);
  }
  
  // draw the robot
  _robotSprite.SetPosition(_robot_pose.x + _window_width/2, _window_height/2 - _robot_pose.y);
  _robotSprite.SetRotation(_robot_pose.theta * 180 / M_PI);
  _window.Draw(_robotSprite);
  
  // display everything
  _window.Display();
}

sf::RenderWindow * rdrawer::RobotDrawer::getWindow(){
  return &_window;
}

void rdrawer::RobotDrawer::zoom(float value){
  this->_camera.Zoom(value);
}
