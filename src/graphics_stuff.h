/*
 * graphics_stuff.h
 *
 *  Created on: Oct 16, 2012
 *      Author: fabrizio
 */
#ifndef GRAPHICS_STUFF_H_
#define GRAPHICS_STUFF_H_
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include <vector>
#include "tools.cpp"

class LandmarksDrawer{
	int smaller_x;
	int smaller_y;
	int greater_x;
	int greater_y;

	cv::Mat * image;

public:
	LandmarksDrawer();
	void drawLandmarks(RobotPosition * rob_pose, std::list<Landmark *> *landmarks);
	void drawLandmarks(RobotPosition * rob_pose, std::vector<Landmark *> *landmarks);	// avoid using this method, this creates a new list containing the elements of the vector and calls the other function (slow!)
	cv::Mat * getImage();
};

/*!
 * drawCross draws a cross of the given size (size = half width) on the image at the given position
 * \param image pointer to the image
 * \param vector with the coordinates
 * \param crossSize size of the cross
 */
static void drawCross(cv::Mat * image, Eigen::Vector3d pos, unsigned int crossSize);

static void drawCross(cv::Mat * image, Eigen::Vector3d pos, unsigned int crossSize, unsigned int channel);

// drawLandmarksImage() now embedded in the LandmarkDrawer Class
//static cv::Mat * drawLandmarksImage(RobotPosition * rob_pose, std::list<Landmark *> *landmarks);

static void drawRobotPose(RobotPosition * rob_pose, int off_x, int off_y, int draw_size, int color, cv::Mat * image);

static void setColor(cv::Vec3b * v, int color);
cv::Scalar getColor(int color_number);

void drawLine(cv::Mat * image, Eigen::Vector2d x1, Eigen::Vector2d x2, unsigned int channel);
void drawLine(cv::Mat * image, Eigen::Vector3d x1, Eigen::Vector3d x2, unsigned int channel);

static void drawAxes(cv::Mat * image, int off_x, int off_y, int size, int color_number);

#endif /* GRAPHICS_STUFF_H_ */
