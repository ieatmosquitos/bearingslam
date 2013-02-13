#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <list>
#include "Eigen/Core"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"

class Observation;
class RobotPosition;

class GraphNode{
 public:
  bool already_in_graph;
  bool id_assigned;
  GraphNode(){this->already_in_graph = false; id_assigned = false;};
  bool hasId(){return id_assigned;};
  void idAssigned(){id_assigned = true;};
};

/*!
 * RobotPosition is a class that indicates a single position of the robot (x,y,Θ).
 * It also has an array of observations, that are the bearings captured from this position.
 */
class RobotPosition : public g2o::VertexSE2, public GraphNode{
 public:
  std::vector<Observation> observations;	// this contains the observations captured from this position

  RobotPosition(double x, double y,double theta);	// constructor
  ~RobotPosition(); // destructor
  
  void addObs(double theta);	// creates an Observation object, and adds it to the Observations vector
  
  double x(){return _estimate[0];};
  double y(){return _estimate[1];};
  double theta(){return _estimate[2];};
  
};

class Observation{
 public:
  RobotPosition* pose;	// pointer to the Position from where the Observation was obtained
  double bearing;			// angle measured

  Observation(double theta, RobotPosition* pose);	// constructor

};

class Landmark : public g2o::VertexPointXY, public GraphNode{
  
  std::vector<Observation*> observations;	// vector of observations that were associated to this landmark
  bool confirmed;		// this tells whether the landmark has been marked as "plausible"
  
 public:
  Landmark();	// constructor
  void addObservation(Observation* toAdd);	// adds an Observation to the set of observations associated with this Landmark
  void estimatePosition();	// this estimates the position of the landmark (given the associated observations) and updates the x,y values of the landmark
  void setPosition(double x, double y);	// this directly sets the position of the landmark
  Eigen::Vector2d * getPosition();
  void getPosition(double * pos);	// puts in pos the position of the Landmark
  void confirm(); // sets the landmark as confirmed
  bool checkConfirmed(int needed);	// checks whether the landmark has at least 'needed' observations, and in the case sets it to confirmed and return TRUE, else return FALSE.
  std::vector<Observation*> * getObservations();	// returns a pointer to the observations vector
  bool isConfirmed();
  unsigned int obsnum;	// redundant counter of the number of observations
  
  double x(){return _estimate[0];};
  double y(){return _estimate[1];};
};

// other useful methods

// returns the absolute value of the given double
static double d_abs(double input);

// Projects the input angle in the [-π, π] interval
static double normalizeAngle(double angle);

// computes the 'distance' between two angles.
// angles are supposed to be expressed in radians, and included in [-π, π].
// this manages the jump between π and -π
// the returned value is positive if the first angle is "on the left" of the second angle (counterclockwise)
static double computeAnglesDifference(double ang1, double ang2);

// returns the angle of the input point, seen from the input pose
static double computeAngle(Eigen::Vector2d* point, RobotPosition* pose);

static double computeAngle(Landmark *, RobotPosition *);

static double degrees2radians(double deg);

static void readTrajFromFile(std::string filename, std::vector<RobotPosition *> * positions);

static Eigen::Matrix3d v2t(Eigen::Vector3d);

static Eigen::Vector3d t2v(Eigen::Matrix3d);

static Eigen::MatrixXd pinv(Eigen::MatrixXd M);

static Eigen::Matrix2d computeRotationMatrix(double radians);

#endif /* TOOLS_H_ */
