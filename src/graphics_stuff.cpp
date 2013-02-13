/*
 * graphics_stuff.cpp
 *
 *  Created on: Oct 16, 2012
 *      Author: fabrizio
 */
#ifndef GRAPHICS_STUFF_CPP_
#define GRAPHICS_STUFF_CPP_

#include "graphics_stuff.h"

#define INFINITE DBL_MAX
#define GRAPHICS_MULTIPLIER 5

LandmarksDrawer::LandmarksDrawer(){
	this->smaller_x = 0;
	this->smaller_y = 0;
	this->greater_x = 0;
	this->greater_y = 0;

	this->image = new cv::Mat(10,10,CV_8UC3, cv::Scalar(255,255,255));
}

void LandmarksDrawer::drawLandmarks(RobotPosition * rob_pose, std::list<Landmark *> *landmarks){
	int min_x=0, max_x=0, min_y=0, max_y=0;
	std::list<Landmark *>::iterator it;
	for(it = landmarks->begin(); it != landmarks->end(); it++){
		if((*it)->getObservations()->size() < 2) continue;
		if((d_abs((*((*it)->getPosition()))[0]) > INFINITE ) || (d_abs((*((*it)->getPosition()))[1]) > INFINITE )){
			//			std::cout << "\nOVER THE INFINITE\n";
			continue;
		}
		Eigen::Vector2d * pos = (*it)->getPosition();
		if((*pos)[0] < min_x) min_x = (int)((*pos)[0]);
		if((*pos)[1] < min_y) min_y = (int)((*pos)[1]);
		if((*pos)[0] > max_x) max_x = (int)((*pos)[0]);
		if((*pos)[1] > max_y) max_y = (int)((*pos)[1]);
	}

	if((max_x == min_x) || (max_y == min_y)){
		cv::Mat * image;
		image = new cv::Mat(10, 10, CV_8UC3, cv::Scalar(255, 255, 200));
		return;
	}

	if(min_x < this->smaller_x){
		this->smaller_x = min_x;
	}
	if(min_y < this->smaller_y){
		this->smaller_y = min_y;
	}
	if(max_x > this->greater_x){
		this->greater_x = max_x;
	}
	if(max_y > this->greater_y){
		this->greater_y = max_y;
	}

	int off_x = -this->smaller_x;
	int off_y = -this->smaller_y;

	int rows = this->greater_y + off_y;
	int cols = this->greater_x + off_x;

	int border_x = (cols) * 0.1;
	int border_y = (rows) * 0.1;

	rows = rows + 2*border_y;
	cols = cols + 2*border_x;

	//	std::cout << "min x: " << min_x << "\tmax x: " << max_x << "\tmin y: " << min_y << "\tmax y: " << max_y << "\n";

	delete this->image;
	this->image = new cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 200));

	Eigen::Vector2d topleft(border_x-1, border_y-1);
	Eigen::Vector2d topright(image->cols - border_x+1, border_y-1);
	Eigen::Vector2d bottomleft(border_x-1, image->rows - border_y+1);
	Eigen::Vector2d bottomright(image->cols - border_x+1, image->rows - border_y+1);

	drawLine(image,topleft,topright,0);
	drawLine(image,topleft,bottomleft,0);
	drawLine(image,bottomleft,bottomright,0);
	drawLine(image,topright,bottomright,0);

	int cross_size = (int) (MIN(cols, rows) / 20);

	for(it = landmarks->begin(); it != landmarks->end(); it++){
		Landmark * lm = *it;
		if (lm->getObservations()->size() < 2) continue;
		if((d_abs((*((*it)->getPosition()))[0]) > INFINITE ) || (d_abs((*((*it)->getPosition()))[1]) > INFINITE )) continue;
		Eigen::Vector3d pos;
		pos[0] = (*(lm->getPosition()))[0] + off_x + border_x;
		pos[1] = (*(lm->getPosition()))[1] + off_y + border_y;
		pos[2] = 1;

		//		std::cout << "lm in: " << pos[0] << "," << pos[1] << "\t";

		if(lm->isConfirmed()){
			//			std::cout << "MAYBE\n";
			drawCross(image, pos, cross_size, 2);
		}
		else{
			//			std::cout << "CONFIRMED\n";
			drawCross(image, pos, cross_size, 4);
		}
	}

	drawAxes(image, off_x + border_x, off_y + border_y, cross_size, 0);

//	RobotPosition scaled_rob_pose(rob_pose->coordinates[0], rob_pose->coordinates[1], rob_pose->theta);
	drawRobotPose(rob_pose,off_x + border_x, off_y + border_y, cross_size, 5, image);

	return;
}

void LandmarksDrawer::drawLandmarks(RobotPosition * rob_pose, std::vector<Landmark *> *landmarks){
	std::list<Landmark *> lms;
	for (unsigned int i=0; i<landmarks->size(); i++){
		lms.push_back((*landmarks)[i]);
	}
	this->drawLandmarks(rob_pose, &lms);
}

cv::Mat * LandmarksDrawer::getImage(){
	return this->image;
}

/*!
 * drawCross draws a cross of the given size (size = half width) on the image at the given position
 * \param image pointer to the image
 * \param vector with the coordinates
 * \param crossSize size of the cross
 */
void drawCross(cv::Mat * image, Eigen::Vector3d pos, unsigned int crossSize) {
	drawCross(image, pos, crossSize, 4);
}

void drawCross(cv::Mat * image, Eigen::Vector3d pos, unsigned int crossSize, unsigned int channel) {
	cv::Vec3b v;

	if(channel == 4){
		v[0] = (uchar)0;
		v[1] = (uchar)0;
		v[2] = (uchar)0;
	}
	else{
		v[0] = (uchar)0;
		v[1] = (uchar)0;
		v[2] = (uchar)0;
		v[channel%3] = (uchar)255;
	}

	image->at<cv::Vec3b>(pos[1], pos[0]) = v;

	for (int offset = 1; offset < crossSize; offset++) {
		double x = pos[0] - offset;
		double y = pos[1] - offset;
		if ((x >= 0) && (x < image->cols) && (y >= 0) && (y < image->rows)) {
			image->at<cv::Vec3b>(y, x) = v;
		}

		x = pos[0] + offset;
		y = pos[1] - offset;
		if ((x >= 0) && (x < image->cols) && (y >= 0) && (y < image->rows)) {
			image->at<cv::Vec3b>(y, x) = v;
		}

		x = pos[0] - offset;
		y = pos[1] + offset;
		if ((x >= 0) && (x < image->cols) && (y >= 0) && (y < image->rows)) {
			image->at<cv::Vec3b>(y, x) = v;
		}

		x = pos[0] + offset;
		y = pos[1] + offset;
		if ((x >= 0) && (x < image->cols) && (y >= 0) && (y < image->rows)) {
			image->at<cv::Vec3b>(y, x) = v;
		}
	}
}

void drawAxes(cv::Mat * image, int off_x, int off_y, int size, int color_number){
	cv::Scalar color = getColor(color_number);
	cv::Point zero(off_x, off_y);
	cv::Point x1(zero.x+size, zero.y);
	cv::Point x2(zero.x-size, zero.y);
	cv::Point y1(zero.x, zero.y+size);
	cv::Point y2(zero.x, zero.y-size);

	cv::line(*image, x1,x2,color,1,0,0);
	cv::line(*image, y1,y2,color,1,0,0);
}


void drawRobotPose(RobotPosition * rob_pose, int off_x, int off_y, int draw_size, int color, cv::Mat * image){
	Eigen::Vector2d p1(draw_size, 0);
	Eigen::Vector2d p2(0, draw_size/2);
	Eigen::Vector2d p3(0, -draw_size/2);
	
	double rpose[3];
	rob_pose->getEstimateData(rpose);
	Eigen::Matrix2d rot = computeRotationMatrix(rpose[2]);

	p1 = rot*p1;
	p2 = rot*p2;
	p3 = rot*p3;

	cv::Point base(off_x + rpose[0],off_y + rpose[1]);
	cv::Point e1(base.x + p1[0], base.y + p1[1]);
	cv::Point e2(base.x + p2[0], base.y + p2[1]);
	cv::Point e3(base.x + p3[0], base.y + p3[1]);

	cv::Scalar color_code = getColor(color);

	cv::line(*image,e1,e2,color_code,1,0,0);
	cv::line(*image,e2,e3,color_code,1,0,0);
	cv::line(*image,e3,e1,color_code,1,0,0);
}

cv::Scalar getColor(int color_number){
	cv::Scalar s;
	color_number = color_number%8;
	if(color_number < 3){
		s[0] = 0;
		s[1] = 0;
		s[2] = 0;
		s[color_number] = 255;
	}
	if(color_number == 3){
		s[0] = 0;
		s[1] = 0;
		s[2] = 0;
	}
	if(color_number == 4){
		s[0] = 255;
		s[1] = 255;
		s[2] = 0;
	}
	if(color_number == 5){
		s[0] = 255;
		s[1] = 0;
		s[2] = 255;
	}
	if(color_number == 6){
		s[0] = 0;
		s[1] = 255;
		s[2] = 255;
	}
	if(color_number == 7){
		s[0] = 255;
		s[1] = 255;
		s[2] = 255;
	}
	return s;
}

void setColor(cv::Vec3b * v, int color){
	color = color%8;
	if(color < 3){
		(*v)[0] = 0;
		(*v)[1] = 0;
		(*v)[2] = 0;
		(*v)[color] = 255;
	}
	if(color == 3){
		(*v)[0] = 0;
		(*v)[1] = 0;
		(*v)[2] = 0;
	}
	if(color == 4){
		(*v)[0] = 255;
		(*v)[1] = 255;
		(*v)[2] = 0;
	}
	if(color == 5){
		(*v)[0] = 255;
		(*v)[1] = 0;
		(*v)[2] = 255;
	}
	if(color == 6){
		(*v)[0] = 0;
		(*v)[1] = 255;
		(*v)[2] = 255;
	}
	if(color == 7){
		(*v)[0] = 255;
		(*v)[1] = 255;
		(*v)[2] = 255;
	}
}

// draws a red line in the given image, at the given coordinates
void drawLine(cv::Mat * image, Eigen::Vector2d x1, Eigen::Vector2d x2, unsigned int channel) {

	if (x1[0] > x2[0]) { // this method is written thinking of x1 at the left of x2, hence this assures it works also in the other case
		drawLine(image, x2, x1, channel);
		return;
	}

	if (x1[0] == x2[0]){
		if(x1[1] > x2[1] ){
			drawLine(image, x2, x1, channel);
			return;
		}
		cv::Vec3b v;

		if(channel<3){
			v[0] = (uchar)0;
			v[1] = (uchar)0;
			v[2] = (uchar)0;
			v[channel] = 255;
		}
		else{
			v[0] = (uchar)0;
			v[1] = (uchar)0;
			v[2] = (uchar)0;
		}
		if(x1[0]<image->cols){
			for(int j=x1[1]; j<x2[1]; j++){
				if(j<image->rows){
					image->at<cv::Vec3b>(j, x1[0]) = v;
				}
			}
		}
		return;
	}

	cv::Vec3b v;

	if(channel<3){
		v[0] = (uchar)0;
		v[1] = (uchar)0;
		v[2] = (uchar)0;
		v[channel] = 255;
	}
	else{
		v[0] = 255;
		v[1] = 255;
		v[2] = 255;
	}

	float slope = (x2[1] - x1[1]) / (x2[0] - x1[0]);
	for (int i = (int) x1[0]; i <= (int) x2[0]; i++) {
		int j = (int) (x1[1] + (i-x1[0]) * slope);
		if((i<image->cols) && (j<image->rows)){
			image->at<cv::Vec3b>(j, i) = v;
		}
	}
}

void drawLine(cv::Mat * image, Eigen::Vector3d x1, Eigen::Vector3d x2, unsigned int channel) {
	Eigen::Vector2d v1(x1[0],x1[1]);
	Eigen::Vector2d v2(x2[0],x2[1]);
	drawLine(image,v1,v2,channel);
}



#endif /* GRAPHICS_STUFF_CPP_ */

