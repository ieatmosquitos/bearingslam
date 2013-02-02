#ifndef RETTA2D_CPP_
#define RETTA2D_CPP_

#include <iostream>
#include "Retta2d.h"

ExplicitLine::ExplicitLine(Eigen::Vector2d * p, double theta){
	// remember, the form is considered to be: ax + by = c
	double m = tan(theta);
	double q = (*p)[1] - m*(*p)[0];

	this->a = -m;
	this->b = 1;
	this->c = q;
}

ExplicitLine::ExplicitLine(RobotPosition * rp, double bearing){
	// similar to ExplicitLine(Eigen::Vector2d * p, double theta), but takes the robotposition and the bearing at which the landmark has been seen
  double pose[3];
  rp->getEstimateData(pose);
  double theta = normalizeAngle(pose[2] + bearing);
	double m = tan(theta);
	double q = pose[1] - m*pose[0];

	this->a = -m;
	this->b = 1;
	this->c = q;
}

Eigen::Vector2d ExplicitLine::intersect(ExplicitLine other_line){
	Eigen::Vector2d ret;
	ret[0] = (other_line.c - this->c) / (other_line.a - this->a);
	ret[1] = this->c - this->a * ret[0];
	return ret;
}

#endif /* RETTA2D_CPP_ */
