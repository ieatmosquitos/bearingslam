#include <iostream>
#include <vector>
#include "tools.cpp"
#include "graphics_stuff.cpp"
#include "Retta2d.cpp"
#include <list>
#include <iostream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <drawer.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

// configs for the first algorithm
#define GENERAL_ANGLE_TOLERANCE	0.2	// this affects the association of a new observation to the previously generated landmarks
#define BEAR_ONLY_ANGLE_TOLERANCE	GENERAL_ANGLE_TOLERANCE/2	//	this affects associations of landmark that still have only one observation
#define SECOND_ANGLE_MIN_DISTANCE	1*GENERAL_ANGLE_TOLERANCE	//	this tells the minimum angle needed for the second closer angle  in order not to be considered ambiguous

// configs for the second algorithm
#define DISTANCE_TOLERANCE 0.2	//	maybe this should be a global variable, that is set up according to the dimension of the space explored by the robot poses

// configs for both algorithms
#define CONFIRM_OBS	10	//	minimum number of consecutive observations needed for the landmark to be confirmed

#define OPTIMIZE_EVERY 50	// every time this number of step has been performed, an optimization phase begins

// usefull variables
bool next_step; 	// when true, the algorithm analyses next step
bool next_optim;	// when true, an optimization step is performed
rdrawer::RobotDrawer * _drawer;	// used for drawing on the screen
std::vector<RobotPosition*> * transformations;  // transformations from a step to the next one
unsigned int next_id;
RobotPosition * last_robot_pose;
Eigen::Matrix3d * odom_info;
Eigen::MatrixXd * obs_info;


// types definition
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

// optimizer declaration
g2o::SparseOptimizer * optimizer;

void handleEvents(sf::RenderWindow * window){
  sf::Event event;
  if(window->GetEvent(event)){
    switch(event.Type){
    case sf::Event::KeyPressed:
      switch(event.Key.Code){
      case sf::Key::N:
      case sf::Key::Return:
	next_step = true;
	break;
      case sf::Key::P:
	_drawer->zoom(1.1);
	break;
      case sf::Key::L:
	_drawer->zoom(0.9);
	break;
      case sf::Key::O:
	next_optim = true;
	break;
      case sf::Key::Escape:
	exit(0);
	break;
      default:
	break;
      }	// end of switch event.key.code
      break;
    case sf::Event::MouseWheelMoved:
      if(event.MouseWheel.Delta<0) _drawer->zoom(1.1f);
      else _drawer->zoom(0.9f);
      break;
    default:
      break;
    }	// end of switch event.type
  }
}

// note: CONCATENATE does side-effect on transf
RobotPosition * concatenate(RobotPosition * step, Eigen::Matrix3d& transf){
  Eigen::Vector3d buff_vect;
  double rstep[3];
  step->getEstimateData(rstep);
  buff_vect << rstep[0], rstep[1], rstep[2];
  
  transf = transf * v2t(buff_vect);
  
  buff_vect = t2v(transf);
  
  RobotPosition * ret = new RobotPosition(buff_vect[0], buff_vect[1], buff_vect[2]);
  
  for(unsigned int obsnum = 0; obsnum<step->observations.size(); obsnum++){
    ret->addObs(step->observations[obsnum].bearing);
  }
  
  return ret;
}

void tryToUnderstand1(RobotPosition * pose,  std::list<Landmark *>* landmarks,std::vector<Landmark *> *  last_observations, std::vector<Landmark *> *  propagate_observations){
  double rpose[3];
  pose->getEstimateData(rpose);
  
  propagate_observations->clear();
  
  std::cout << "Robot Position:\t" << rpose[0] << "\t" << rpose[1] << "\t" << rpose[2] << "\n";
  
  double expected[last_observations->size()];	//	this will contain the projections of the previously seen landmarks in the current pose
  // I.E. this tells where we expect to see the landmarks propagated from the last step

  bool use_general_angle_tolerance[last_observations->size()];	// this will tell if the angle tolerance to be used is the general one or the bearing-only based one

  int associations[pose->observations.size()];	// this will contain the candidate associations, that will become effective ONLY IF there will be no ambiguity
  for(unsigned int i=0; i<pose->observations.size(); i++){
    associations[i] = -1;
  }

  std::cout << "We expect to see stuff at:\n";
  for(unsigned int i = 0; i<last_observations->size(); i++){	// populate the expected array
    Landmark * lm = (*last_observations)[i];

    if(lm->getObservations()->size() > 1){	// first case: landmark already has an estimated position
      expected[i] = computeAngle(lm, pose);
      use_general_angle_tolerance[i] = true;
    }
    else{	// second case: landmark has been seen only once up to now, hence there is no position estimation
      double prev_seen = (*(lm->getObservations()))[0]->bearing;
      double prev_theta = (*(lm->getObservations()))[0]->pose->theta();
      double rotated = computeAnglesDifference(pose->theta(), prev_theta);
      expected[i] = prev_seen-rotated;
      use_general_angle_tolerance[i] = false;
    }

    std::cout << expected[i] << "\n";
  }

  std::cout << "observations:\n";
  for(unsigned int obs_counter=0; obs_counter < pose->observations.size(); obs_counter++){	// for every observation in the current position
    std::cout << (pose->observations)[obs_counter].bearing << ": ";
    // Try to assign it to a previously generated landmark
    if(last_observations->size() > 0){

      unsigned int closer_index;
      unsigned int second_closer_index;
      // compute closer_index and second_closer_index.

      double distances[last_observations->size()];
      for(unsigned int i=0; i < last_observations->size(); i++){
	distances[i] = d_abs(computeAnglesDifference(expected[i], pose->observations[obs_counter].bearing));
      }	//	now 'distances' contains the distance between the currently considered observation and the expected values

      closer_index = 0;
      if(last_observations->size() == 1){
	second_closer_index = 0;
      }
      else{
	second_closer_index = 1;
	if(distances[1] < distances[0]){
	  closer_index = 1;
	  second_closer_index = 0;
	}
      }
      for (unsigned int i = 1; i < last_observations->size(); i++){
	if (distances[i] <  distances[closer_index]){
	  second_closer_index = closer_index;
	  closer_index = i;
	}
	else{
	  if(distances[i] <  distances[second_closer_index]){
	    second_closer_index = i;
	  }
	}
      }
      // if closer is "close enough" and second_closer is "far enough" (I.E. there is no ambiguity)
      // associate the current observation to 'closer'.
      double tolerance = GENERAL_ANGLE_TOLERANCE;
      if(!use_general_angle_tolerance[closer_index]){
	tolerance = BEAR_ONLY_ANGLE_TOLERANCE;
      }
      std::cout << "tolerance is " << tolerance << "\n";
      if(distances[closer_index] < tolerance){
	if((second_closer_index == closer_index) || (distances[second_closer_index] > SECOND_ANGLE_MIN_DISTANCE)){
	  associations[obs_counter] = closer_index;
	  std::cout << "close to " << expected[closer_index] << "\n";
	}
	else{
	  std::cout << "close to " << expected[closer_index] << " but also to " << expected[second_closer_index] << "\n";
	}
      }
      else{
	std:: cout << "is far from everything\n";
      }

      // NOTE:	Do not add the observation to the set inside the landmark corresponding to
      //			'closer' yet, because, if two or more NEW observations are associated to the same
      //			landmark, none of them will be added.
    }
  }

  // add the computed observations to the landmarks, but only if there is no ambiguity

  bool associated[last_observations->size()];	//	associated[i] will be true if there is at least 1 new observation associated to the i-th landmark in last_observations
  bool ambiguous[last_observations->size()];	//	ambiguous[i] will be true if there are at least 2 new observations associated to the i-th landmark in last_observations
  // initialize the arrays just created
  for(unsigned int i=0; i<last_observations->size(); i++){
    associated[i] = false;
    ambiguous[i] = false;
  }

  // tell which ones are ambiguous
  for(unsigned int i=0; i < pose->observations.size(); i++){
    unsigned int value = associations[i];
    if(value!=-1){
      if (!associated[value]){
	associated[value] = true;
      }
      else{	//	associated is true, hence there is already a new observation (or more) polling for the landmark
	ambiguous[value] = true;
      }
    }
  }

  // add non-ambiguous observations to the corresponding landmarks
  // and create new landmarks for the new observed unassociated observations
  std::cout << "adding observations to landmarks\n";
  for(unsigned int i=0; i < pose->observations.size(); i++){
    std::cout << "obs " << i << " ";
    if((associations[i] != -1) && (associated[associations[i]])){
      if(!ambiguous[associations[i]]){
	std::cout << "tailed to " << associations[i] << "\n";
	Landmark * lm = (*last_observations)[associations[i]];
	lm->addObservation(&(pose->observations[i]));
	lm->estimatePosition();
	lm->checkConfirmed(CONFIRM_OBS);
	double newpos[2];
	(*last_observations)[associations[i]]->getPosition(newpos);
	std::cout << "\tnew position estimated:\t" << newpos[0] << "\t" << newpos[1] << "\n";
	propagate_observations->push_back((*last_observations)[associations[i]]);
      }
      else{
	std::cout << "should be tailed to " << associations[i] << ", but there is ambiguity\n";
	// TODO:	add here instructions for ambiguous observations
	//			maybe propagate the ambiguities to the successive step, without adding the observations to the set
      }
    }
    else{	// unassociated
      std::cout << "is a new landmark\n";
      Landmark * lm = new Landmark();
      lm->addObservation (&(pose->observations[i]));
      //			lm->estimatePosition();
      landmarks->push_back(lm);
      propagate_observations->push_back(lm);
    }
  }

  std::cout << "now checking landmarks from previous step\n";
  for(unsigned int i = 0; i<last_observations->size(); i++){
    std::cout << "prev_obs " << i << " ";
    Landmark * lm = (*last_observations)[i];
    if(!associated[i]){
      // for the landmarks expected to be seen that are not in the current set of observations, we have two cases:
      if(lm->getObservations()->size() > CONFIRM_OBS){
	// case one: the landmark has at least CONFIRM_OBS observations, then it must be confirmed
	std::cout << "was old enough, CONFIRMED\n";
	lm->confirm();
      }
      else{
	// case two: the landmark has less than CONFIRM_OBS observations, then it is trashed
	std::cout << "was too young, REMOVED\n";
	landmarks->remove(lm);
	delete lm;
      }
    }
    else{
      if(ambiguous[i]){
	std::cout << "was ambiguous, PROPAGATED as it is\n";
	propagate_observations->push_back(lm);
      }
      else{
	std::cout << "has been REINFORCED\n";
      }
    }
  }
  std::cout << "\n\n";
}

void printState(std::vector<Landmark*> * landmarks, RobotPosition * pose, rdrawer::RobotPose * drawer_robot_pose){
  for(unsigned int i=0; i < landmarks->size(); i++){
    Landmark * l = (*landmarks)[i];
    if(l->obsnum > 1){	// it's not a newborn
      if(l->isConfirmed()){	// it is considered "reliable"
	_drawer->addAndCreateLandmark(l->x(), l->y());
      }
      else{	// it still has been seen too few times
	_drawer->addAndCreateUnconfirmedLandmark(l->x(), l->y());
      }   
    }
  }
  drawer_robot_pose->x = pose->x();
  drawer_robot_pose->y = pose->y();
  drawer_robot_pose->theta = pose->theta();
  _drawer->setRobotPose(*drawer_robot_pose);
}

void printState(std::list<Landmark*> * landmarks, RobotPosition * pose, rdrawer::RobotPose * drawer_robot_pose){
  std::list<Landmark *>::iterator it;
  for(it = landmarks->begin(); it != landmarks->end(); it++){
    if((*it)->obsnum > 1){	// it's not a newborn
      if((*it)->isConfirmed()){	// it is considered "reliable"
	_drawer->addAndCreateLandmark((*it)->x(), (*it)->y());
      }
      else{	// it still has been seen too few times
	_drawer->addAndCreateUnconfirmedLandmark((*it)->x(), (*it)->y());
      }
    }
  }
  drawer_robot_pose->x = pose->x();
  drawer_robot_pose->y = pose->y();
  drawer_robot_pose->theta = pose->theta();
  _drawer->setRobotPose(*drawer_robot_pose);
}

void init(){
  std::cout << "Initializing stuff" << std::endl;
  
  next_id = 0;
  
  odom_info = new Eigen::Matrix3d();
  (*odom_info) <<	500,	0,	0,
    			0,	500,	0,
    			0,	0,	100;

  obs_info = new Eigen::MatrixXd(1,1);
  (*obs_info) <<	500;
  
  
  // allocating the optimizer
  optimizer = new g2o::SparseOptimizer();
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
  
  optimizer->setAlgorithm(solver);
  optimizer->setVerbose(true);
  
}

void populateGraph(std::vector<RobotPosition *> * poses, std::list<Landmark *> * landmarks){
  std::cout << "clearing optimizer..." << std::endl;
  // optimizer->clear();
  
  // put poses in the graph
  for(unsigned int i=0; i<poses->size(); i++){
    std::cout << "adding pose " << i;
    // add this pose to the graph
    optimizer->addVertex((*poses)[i]);
    std::cout << "...done." << std::endl;
    
    if(i>0){
      // add the constraint regarding the rototranslation performed from the previous step
      g2o::EdgeSE2 * odom_edge = new g2o::EdgeSE2;
      odom_edge->vertices()[0] = (*poses)[i-1];
      odom_edge->vertices()[1] = (*poses)[i];
      std::cout << "accessing transformations informations" << std::endl;
      g2o::SE2 * measurement = new g2o::SE2((*transformations)[i]->x(), (*transformations)[i]->y(), (*transformations)[i]->theta());
      odom_edge->setMeasurement(*measurement);
      odom_edge->setInformation(*odom_info);
      optimizer->addEdge(odom_edge);
    }
  }
  
  // put landmarks in the graph
  std::list<Landmark *>::iterator iter;
  for(iter = landmarks->begin(); iter!=landmarks->end(); iter++){
    
    Landmark * lm = *iter;
    
    std::cout << "if(!lm->isConfirmed())..." << std::endl;
    if(!lm->isConfirmed()){
      continue;
    }
    
    if(!lm->hasId()){
      lm->setId(next_id++);
      lm->idAssigned();
    }
    
    optimizer->addVertex(lm);
    
    for(unsigned int o=0; o<lm->getObservations()->size(); o++){
      Observation * observation= (*(lm->getObservations()))[o];
      RobotPosition * pose = observation->pose;
      
      g2o::EdgeSE2PointXYBearing* obs_edge =  new g2o::EdgeSE2PointXYBearing;
      obs_edge->vertices()[0] = pose;
      obs_edge->vertices()[1] = lm;
      obs_edge->setMeasurementData(&(observation->bearing));
      obs_edge->setInformation(*obs_info);
      
      optimizer->addEdge(obs_edge);
    }
  }
  
  optimizer->vertex(0)->setFixed(true);
  optimizer->initializeOptimization();
}

// delete unconfirmed landmarks from the given landmarks list. buffer is used as a buffer and its contents are cleared.
void deleteUnconfirmedLandmarks(std::list<Landmark *> * landmarks, std::vector<Landmark *> * buffer){
  
  buffer->clear();
  std::list<Landmark *>::iterator iter;
  for(iter = landmarks->begin(); iter!=landmarks->end(); iter++){
    Landmark * lm = *iter;
    
    if(lm->isConfirmed()){
      buffer->push_back(lm);
    }
  }
  
  landmarks->clear();
  
  for(unsigned int i=0; i<buffer->size(); i++){
    landmarks->push_back((*buffer)[i]);
  }
  buffer->clear();
}

int main(int argc, char** argv){
  std::cout << "LANDMARK ESTIMATOR" << std::endl;
  std::cout << "==================" << std::endl;
  
  // check the input
  if(argc < 2){
    std::cout << "Usage: LadmarksEstimator <traj_file>\n";
    exit(0);
  }
  
  init();
  
  // create structures for storing datas
  std::vector<RobotPosition*> poses;  // absolute poses
  transformations = new std::vector<RobotPosition *>;
  std::vector<Landmark *> landmarks;	// this will contain, at the end, all the confirmed landmarks
  std::list<Landmark *> tmp_landmarks;	// this is a buffer
  
  // add the first node
  transformations->push_back(new RobotPosition(0,0,0));
  
  // load other nodes from file
  readTrajFromFile(std::string(argv[1]), transformations);
  
  if(transformations->size() < 1){
    std::cout << "no infos loaded... quitting" << std::endl;
    exit(0);
  }
  
  std::string graphname = basename(argv[1]);
  graphname = graphname.substr(0,graphname.length()-4);
  std::ofstream g2oout((graphname+(std::string("_graph.g2o"))).c_str());
  
  _drawer = new rdrawer::RobotDrawer();
  rdrawer::RobotPose drawer_robot_pose;

  //<<<<<<<<<<<BEGIN OF ALGORITHM 1 LAUNCHER
  std::vector<Landmark *> buff1;
  std::vector<Landmark *> buff2;
  bool buffswitch = true;
  Eigen::Matrix3d buff_transf;
  buff_transf <<	1, 0, 0,
    			0, 1, 0,
    			0, 0, 1;
  
  unsigned int loop_iterations = 0;
  
  for (unsigned int i=0; i<transformations->size(); i++){
    
    loop_iterations ++;
    next_step = false;
    
    // add the new pose to the vector
    if(i!=0){
      buff_transf = r2t(poses[i-1]);
    }
    poses.push_back(concatenate((*transformations)[i], buff_transf));
    last_robot_pose = poses[i];
    
    // to assign an ID
    poses[i]->setId(next_id++);
    poses[i]->idAssigned();
    
    std::cout << "\nstep " << i+1 << "\n";
    if(buffswitch){
      tryToUnderstand1(poses[i], &tmp_landmarks, &buff1, &buff2);
    }
    else{
      tryToUnderstand1(poses[i], &tmp_landmarks, &buff2, &buff1);
    }
    buffswitch = !buffswitch;
    
    if(loop_iterations > OPTIMIZE_EVERY){	// time to optimize
      std::cout << "time to optimize" << std::endl;
      loop_iterations = 0;
      buff1.clear();
      buff2.clear();
      deleteUnconfirmedLandmarks(&tmp_landmarks, &buff1);
      
      populateGraph(&poses, &tmp_landmarks);
      
      optimizer->optimize(10);
    }
    
    std::cout << "creating image...\n";
    printState(&tmp_landmarks, poses[i], &drawer_robot_pose);
    
    std::cout << "displaying image...\n";
    while(!next_step){
      handleEvents(_drawer->getWindow());
      _drawer->draw();
      usleep(200);
    }
    _drawer->clearAndDeleteLandmarks();
    _drawer->clearAndDeleteUnconfirmedLandmarks();

    // cvWaitKey(0);
  }
  
  buff1.clear();
  buff2.clear();
  deleteUnconfirmedLandmarks(&tmp_landmarks, &buff1);
  
  populateGraph(&poses, &tmp_landmarks);
  optimizer->optimize(10);
  
  
  std::cout << "program terminated! You can now keep optimizing optimize (o) or exit (esc)" << std::endl;
  next_optim = false;
  while(true)	// exit is performed inside handleEvents method
  {
    if(next_optim){
      next_optim = false;
      optimizer->optimize(10);
      // optimizer->save("graph_after.g2o");
    }
    printState(&tmp_landmarks, last_robot_pose, &drawer_robot_pose);
    while(!next_optim){
      handleEvents(_drawer->getWindow());
      _drawer->draw();
      usleep(200);
    }
    _drawer->clearAndDeleteLandmarks();
    _drawer->clearAndDeleteUnconfirmedLandmarks();
  }
  
  //>>>>>>>>>>>END OF ALGORITHM 1 LAUNCHER

  // // should now remove the landmarks that still have been seen fewer than CONFIRM_OBS times
  // std::cout << "algorithm finished, purging the landmarks list (CONFIRM_OBS is " << CONFIRM_OBS << ")\n";
  // std::list<Landmark *>::iterator iter;
  // for(iter = tmp_landmarks.begin(); iter!=tmp_landmarks.end(); iter++){
  //   Landmark * lm = *iter;
  //   std::cout << "landmark in:\t" << lm->x() << "\t"<< lm->y() <<" has " << lm->obsnum << " observations, ";
  //   if(lm->isConfirmed()){
  //     std::cout << "CONFIRMED\n";
  //     landmarks.push_back(lm);
  //   }
  //   else{
  //     std::cout << "IGNORED\n";
  //   }
  // }

  // // at this point, 'landmarks' will contain the landmarks estimated positions
  // std::cout << "\nlandmarks at:\n";
  // std::list<Landmark *>::iterator it;
  // for(unsigned int i=0; i< landmarks.size(); i++){
  //   Landmark * lm = landmarks[i];
  //   double pos[2];
  //   lm->getPosition(pos);
  //   double x = floorf((pos[0] * 1000) + 0.5)/1000;
  //   double y = floorf((pos[1] * 1000) + 0.5)/1000;
  //   std::cout << x << "  " << y << "\n";
  // }
  
  // // generate output g2o file
  // std::cout << "generating output file for g2o" << std::endl;
  // // generate poses nodes and edges between them
  // // g2oout << "VERTEX_SE2 " << next_id++ << " 0 0 0" << std::endl;	// very first position
  // for(unsigned int i=0; i<poses.size(); i++){
  //   RobotPosition * pose = poses[i];
  //   RobotPosition * transf = (*transformations)[i];
  //   RobotPosition * prev_pose;
  //   // unsigned int prev_id;
    
  //   if(!pose->hasId()){
  //     pose->setId(next_id++);
  //     pose->idAssigned();
  //   }
    
  //   g2oout << "VERTEX_SE2 " << pose->id() << " " << pose->x() << " " << pose->y() << " " << pose->theta() << std::endl;
  //   optimizer->addVertex(pose);
    
  //   if(i>0){
  //     prev_pose = poses[i-1];
  //     g2oout << "EDGE_SE2 " << prev_pose->id() << " " << pose->id() << " " << transf->x() << " " << transf->y() << " " << transf->theta() << " 500 0 0 500 0 100" << std::endl;
  //   }
  // }
  // // generate landmarks nodes and edges towards them from the poses where they were seen from
  // for(unsigned int l=0; l < landmarks.size(); l++){
  //   Landmark * lmark = landmarks[l];
  //   if(!lmark->hasId()){
  //     lmark->setId(next_id++);
  //     lmark->idAssigned();
  //   }
  //   // add the landmark
  //   g2oout << "VERTEX_XY " << lmark->id() << " " << lmark->x() << " " << lmark->y() << std::endl;
  //   optimizer->addVertex(lmark);
    
  //   // add the associated bearing constraints
  //   for(unsigned int o=0; o<lmark->getObservations()->size(); o++){
  //     Observation * observation= (*(lmark->getObservations()))[o];
  //     RobotPosition * pose = observation->pose;
      
  //     g2oout << "EDGE_BEARING_SE2_XY " << pose->id() << " " << lmark->id() << " " << observation->bearing << " 200" << std::endl;
      
  //     g2o::EdgeSE2PointXYBearing* obs_edge =  new g2o::EdgeSE2PointXYBearing;
  //     obs_edge->vertices()[0] = pose;
  //     obs_edge->vertices()[1] = lmark;
  //     obs_edge->setMeasurementData(&(observation->bearing));
  //     obs_edge->setInformation(*obs_info);
  //     optimizer->addEdge(obs_edge);
  //   }
  // }
  
  // optimizer->save("graph_before.g2o");
  
  // populateGraph(&poses, &tmp_landmarks);
  
  // std::cout << "program terminated! You can now optimize (o) or exit (esc)" << std::endl;
  
  // optimizer->vertex(0)->setFixed(true);
  // optimizer->setVerbose(true);
  // optimizer->initializeOptimization();
  
  // next_optim = false;
  // while(true)	// exit is performed inside handleEvents method
  // {
  //   if(next_optim){
  //     next_optim = false;
  //     optimizer->optimize(10);
  //     // optimizer->save("graph_after.g2o");
  //   }
  //   printState(&tmp_landmarks, last_robot_pose, &drawer_robot_pose);
  //   while(!next_optim){
  //     handleEvents(_drawer->getWindow());
  //     _drawer->draw();
  //     usleep(200);
  //   }
  //   _drawer->clearAndDeleteLandmarks();
  //   _drawer->clearAndDeleteUnconfirmedLandmarks();
  // }
  
  return 0;
}
