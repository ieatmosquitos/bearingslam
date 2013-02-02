#ifndef TOOLS_CPP_
#define TOOLS_CPP_

#include <iostream>
#include "tools.h"
#include "Retta2d.h"
#include <math.h>
#include "FileReader.cpp"
#include <cstdlib>
#include "Retta2d.cpp"
#include "Eigen/Dense"


using namespace Eigen;
/*
Coordinates::Coordinates(double x, double y){
	this->x=x;
	this->y=y;
}

Coordinates::Coordinates(){
	this->x=0;
	this->y=0;
}
 */

RobotPosition::RobotPosition(double x, double y, double theta){
  _estimate = g2o::SE2(x, y, theta);
}

RobotPosition::~RobotPosition(){
}

void RobotPosition::addObs(double bearing){
	Observation newobservation(bearing,this);
	this-> observations.push_back(newobservation);
}

Observation::Observation(double bearing,RobotPosition* pose){
	this->bearing=bearing;
	this->pose=pose;
}

Landmark::Landmark(){
	this->confirmed = false;
	this->obsnum = 0;
	_estimate[0] = 0;
	_estimate[1] = 0;
}

void Landmark::addObservation(Observation * toAdd){
	this->observations.push_back(toAdd);
	this->obsnum++;
}

std::vector<Observation *> * Landmark::getObservations(){
	return &(this->observations);
}

Eigen::Vector2d * Landmark::getPosition(){
  Eigen::Vector2d * ret = new Eigen::Vector2d(_estimate[0], _estimate[1]);
  return ret;
}

void Landmark::getPosition(double * pos){
  pos[0] = _estimate[0];
  pos[1] = _estimate[1];
}

void Landmark::setPosition(double x, double y){
	_estimate[0] = x;
	_estimate[1] = y;
}

void Landmark::confirm(){
	this->confirmed = true;
}

void Landmark::checkConfirmed(int needed){
	if(this->observations.size() > needed){
		this->confirm();
	}
}

bool Landmark::isConfirmed(){
	return this->confirmed;
}

void Landmark::estimatePosition(){
	// IF the observations are only 2, gets the intersection
	if(this->getObservations()->size()==2){
		//		Retta2d r1((*(this->getObservations()))[0]->pose, (*(this->getObservations()))[0]->bearing);
		//		Retta2d r2((*(this->getObservations()))[1]->pose, (*(this->getObservations()))[1]->bearing);
		//
		//		Eigen::Vector2d* inter = Retta2d::getIntersezione(&r1,&r2);
		//		(*(this->getPosition()))[0] = (*inter)[0];
		//		(*(this->getPosition()))[1] = (*inter)[1];
		//		delete inter;

		ExplicitLine r1((*(this->getObservations()))[0]->pose, (*(this->getObservations()))[0]->bearing);
		ExplicitLine r2((*(this->getObservations()))[1]->pose, (*(this->getObservations()))[1]->bearing);
		Eigen::Vector2d inter = r1.intersect(r2);
		
		_estimate[0] = inter[0];
		_estimate[1] = inter[1];
		return;
	}

	// ELSE
	// compute the minimum norm distance from the lines obtained given the RobotPositions and the relative bearings
	// The system is Av = b			with	v = <x,y>
	// hence mean_v = pinv(A)*b		with	pinv(A)=((A'*A)^-1)*A'
	Eigen::MatrixXd A(this->getObservations()->size(),2);
	Eigen::VectorXd b(this->getObservations()->size());
	for(unsigned int i=0; i<this->getObservations()->size(); i++){
		Observation * obs = (*(this->getObservations()))[i];
		ExplicitLine e(obs->pose, obs->bearing);
		A(i,0) = e.a;
		A(i,1) = e.b;
		b(i,0) = e.c;
	}

	//	Eigen::VectorXd mean_v;
	// 	A.lu().solve(b, &mean_v);
	////
	//	LU<MatrixXd> luOfA(A);
	//	luOfA.solve(b, &mean_v);
	Eigen::VectorXd mean_v = pinv(A) * b;

	_estimate[0] = mean_v[0];
	_estimate[1] = mean_v[1];
}

double d_abs(double input){
	return (input < 0 ? -input : input);
}

double normalizeAngle(double angle) {
	double absangle = (angle > 0 ? angle : -angle); // absolute value of angle

	// now, bring the angle in the interval [-2pi, 2pi]
	if (absangle > 2 * M_PI) {
		if (angle > 0) { // must decrease the value
			while (angle > 2 * M_PI) angle -= 2 * M_PI; // any better idea to do the module operation between two doubles?
		} else { // must increase the value
			while (angle < -2 * M_PI) angle += 2 * M_PI;
		}
	}

	// now, bring the angle in the [-pi, pi] interval
	absangle = (angle > 0 ? angle : -angle);
	if (absangle > M_PI) {
		(angle > 0 ? angle -= 2 * M_PI : angle += 2 * M_PI);
	}
	return angle;
}

double computeAnglesDifference(double ang1, double ang2){
	if(d_abs(ang1-ang2) > M_PI){	// this passes through the discontinuous point π
		if(ang1 > 0){	// ang2 is negative
			return -(2*M_PI - (ang1-ang2));
		}
		else{	// ang1 negative, ang2 positive
			return (2*M_PI - (ang2-ang1));
		}
	}
	return (ang1-ang2);
}

double computeAngle(Landmark * lmark, RobotPosition* rpose){
  double pose[3];
  double point[2];
  rpose->getEstimateData(pose);
  lmark->getPosition(point);
  double dx = point[0] - pose[0];
  double dy = point[1] - pose[1];
  double pure_angle = atan2(dy,dx);
  
  return computeAnglesDifference(pure_angle, pose[2]);
}

double computeAngle(Eigen::Vector2d * point, RobotPosition * rpose){
  double pose[3];
  rpose->getEstimateData(pose);
  double dx = (*point)[0] - pose[0];
  double dy = (*point)[1] - pose[1];
  double pure_angle = atan2(dy,dx);
  
  return computeAnglesDifference(pure_angle, pose[2]);
}

double degrees2radians(double deg){
	return deg/180*M_PI;
}

void readTrajFromFile(std::string filename, std::vector<RobotPosition*> * positions){
	FileReader fr(filename);

	if(!fr.is_open()){
		std::cout << "there is no " << filename << " file, or it cannot be read\n";
		return;
	}

	std::vector<std::string> textline;
	fr.readLine(&textline);
	while(fr.good()){
		if(textline.size() >= 3){
			double x = atof(textline[0].c_str());
			double y = atof(textline[1].c_str());
			double theta = normalizeAngle(atof(textline[2].c_str()));

			RobotPosition * robpos = new RobotPosition(x,y,theta);

			for (unsigned int i = 3; i<textline.size(); i++){
				double bearing = normalizeAngle(atof(textline[i].c_str()));
				robpos->addObs(bearing);
			}

			positions->push_back(robpos);
		}
		textline.clear();
		fr.readLine(&textline);
	}
}

Eigen::MatrixXd pinv(Eigen::MatrixXd M){
	return ((M.transpose()*M).inverse()*M.transpose());
}

Eigen::Matrix2d computeRotationMatrix(double radians){
	Eigen::Matrix2d ret;
	ret(0,0) = cos(radians);
	ret(0,1) = -sin(radians);
	ret(1,0) = sin(radians);
	ret(1,1) = cos(radians);
	return ret;
}

static Eigen::Matrix3d v2t(Eigen::Vector3d v){
  Eigen::Matrix3d ret;
  ret <<
    cos(v[2]),	-sin(v[2]),	v[0],
    sin(v[2]),	cos(v[2]),	v[1],
    0,		0,		1;
  return ret;
}

static Eigen::Vector3d t2v(Eigen::Matrix3d m){
  Eigen::Vector3d ret;
  ret[0] = m(0,2);
  ret[1] = m(1,2);
  ret[2] = atan2(m(1,0),m(0,0));
  return ret;
}

#endif /* TOOLS_CPP_ */
