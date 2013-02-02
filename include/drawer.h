#ifndef ROBDRAWER_H
#define ROBDRAWER_H

#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"

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

    RobotPose _robot_pose;
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
    void clearLandmarksList();
    void clearAndDeleteLandmarks();
    void clearUnconfirmedLandmarksList();
    void clearAndDeleteUnconfirmedLandmarks();
    void draw();
    sf::RenderWindow * getWindow();
    void zoom(float value);
  };

}

#endif  // ROBDRAWER_H
