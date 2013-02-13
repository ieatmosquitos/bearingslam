/*
 * testRette.cpp
 *
 *  Created on: Sep 24, 2012
 *      Author: gwala
 */

#include <iostream>
#include "Retta2d.cpp"
#include "tools.cpp"
#include <math.h>
#include <cstdlib>
using namespace std;

int main(int argc, char** argv){

	/*
	Eigen::Vector2d p1 = Eigen::Vector2d(0,0);
	Retta2d r1 = Retta2d(&p1,M_PI/2);

	Eigen::Vector2d p2 = Eigen::Vector2d(1,0);
	Retta2d r2 = Retta2d(&p2,M_PI/4);

	Eigen::Vector2d* intersez;
	intersez = Retta2d::getIntersezione(&r1,&r2);

	Retta2d r = Retta2d::rettaPerDuePunti(&p1,&p2);

	if (intersez == NULL) exit(1);

	cout << r.toString() << "\n";
	//cout << "Intersezione x: " << (*intersez)(0) << " y: " << (*intersez)	(1) << "\n";
	*/
	Eigen::Vector2d p1 = Eigen::Vector2d(1,1);
	Eigen::Vector2d p2 = Eigen::Vector2d(3,2);
	Retta2d r1 = Retta2d::rettaPerDuePunti(&p1,&p2);

	cout << r1.toString() << "\n";

	Eigen::Vector2d p3 = Eigen::Vector2d(3,-2);
	Retta2d r2 = Retta2d::rettaBearing(&p3,M_PI/4);

	cout << r2.toString() << "\n";

	Eigen::Vector2d* intersez;
	intersez = Retta2d::getIntersezione(&r1,&r2);
	cout << "Intersezione x: " << (*intersez)(0) << " y: " << (*intersez)(1) << "\n";

	p3 = Eigen::Vector2d(3,0);
	Retta2d r3 = Retta2d::rettaOrtogonalePerPunto(&r1,&p3);

	intersez = Retta2d::getIntersezione(&r1,&r3);
	cout << "Intersezione x: " << (*intersez)(0) << " y: " << (*intersez)(1) << "\n";

	std::cout << "\n\n";

	RobotPosition rp1(1,0,0);
	ExplicitLine e1(&rp1, M_PI_2);
	std::cout << "Line e1:\t" << e1.a << "x + " << e1.b << "y = " << e1.c << "\n";
}



