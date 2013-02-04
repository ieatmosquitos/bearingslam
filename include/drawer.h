#ifndef ROBDRAWER_H
#define ROBDRAWER_H

#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"
#include "Eigen/Core"

#include <iostream>
#include <fstream>

namespace rdrawer{ 

  struct Landmark{
    double x;
    double y;
  
    Landmark(double x, double y) {this->x = x; this->y = y;};
  };

  struct RobotPose{
    double x;
    double y;
    double theta;
  };

  class RobotDrawer{
    int _window_width;
    int _window_height;
  
    sf::RenderWindow _window;
    sf::View _camera;
    sf::Rect<float> _camera_rect;
    sf::Image _backgroundImage;
    sf::Sprite _backgroundSprite;
    sf::Image _robotImage;
    sf::Sprite _robotSprite;
    sf::Image _landmarkImage;
    sf::Sprite _landmarkSprite;
    sf::Image _unconf_landmarkImage;
    sf::Sprite _unconf_landmarkSprite;
    sf::Image _traj_stepImage;
    sf::Sprite _traj_stepSprite;

    RobotPose _robot_pose;
    std::vector<Eigen::Vector3d> trajectory;
    std::vector<rdrawer::Landmark *> landmarks;
    std::vector<rdrawer::Landmark *> unconfirmed_landmarks;
  
  public:
    RobotDrawer();
    ~RobotDrawer();
    void setRobotPose(RobotPose r);
    void addLandmark(Landmark *);
    void addAndCreateLandmark(double x, double y);
    void addUnconfirmedLandmark(Landmark *);
    void addAndCreateUnconfirmedLandmark(double x, double y);
    void addTrajectoryStep(double x, double y, double theta);
    
    void clearLandmarksList();
    void clearAndDeleteLandmarks();
    void clearUnconfirmedLandmarksList();
    void clearAndDeleteUnconfirmedLandmarks();
    void clearTrajectory();
    void clearAll();
    void draw();
    sf::RenderWindow * getWindow();
    void zoom(float value);
  };

}

#endif  // ROBDRAWER_H
