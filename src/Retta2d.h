#ifndef RETTA2D_H_
#define RETTA2D_H_

#include <iostream>
#include <math.h>
#include "tools.h"
#include <eigen3/Eigen/Core>

class ExplicitLine{
	// the form is considered to be ax + by = c
public:
	double a;
	double b;
	double c;

	ExplicitLine(Eigen::Vector2d * p, double theta);
	ExplicitLine(RobotPosition * rp, double bearing);
	Eigen::Vector2d intersect(ExplicitLine other_line);
};


#endif /* RETTA2D_H_ */
