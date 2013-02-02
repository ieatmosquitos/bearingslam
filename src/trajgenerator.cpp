/*
 * trajgenerator.cpp
 * This program takes as input a text file containing:
 * 		robot poses	(x, y, theta)
 * 		landmarks positions (x,y)
 *
 * Every line containing 3 elements will be considered as a RobotPosition
 * Every line containing 2 elements will be consodered as a landmark
 *
 * The output will be a file describing robot positions and bearings where the landmarks are seen from that robot position
 *
 * NOTE:	Every landmark will be seen from every robot position.
 * 			If you want an incomplete graph, you'll have to remove some bearings from the output file
 *
 *  Created on: Oct 14, 2012
 *      Author: fabrizio
 */

#include <vector>
#include "tools.cpp"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

inline std::string stringify(double x)
{
  std::ostringstream o;
  if (!(o << x)){
	  //    throw BadConversion("stringify(double)");
	  std::cout << "Bad Conversion: stringify(double)\n";
  }
  return o.str();
}

int main(int argc, char** argv){

	if(argc<2){
		std::cout << "Usage: TrajGenerator <input file name>\n";
		exit(0);
	}

	std::cout << "\n<<<<<<<<< Tajectory Generator >>>>>>>>>\n\n";
	std::string inputname(argv[1]);
	FileReader fin(argv[1]);
	if(!fin.is_open()){
		std::cout << "cannot read file\n" << argv[1];
		exit(1);
	}

	std::vector<RobotPosition> poses;
	std::vector<Eigen::Vector2d> landmarks;

	std::vector<std::string> textline;
	fin.readLine(&textline);
	while (fin.good()) {
		if (textline.size() == 3) {	// this is a RobotPosition
			double x = atof(textline[0].c_str());
			double y = atof(textline[1].c_str());
			double theta = atof(textline[2].c_str());

			poses.push_back(RobotPosition(x,y,theta));
		}
		else{
			if (textline.size() == 2) {	// this is a landmark
				double x = atof(textline[0].c_str());
				double y = atof(textline[1].c_str());

				Eigen::Vector2d v(x,y);

				landmarks.push_back(v);
			}
		}
		textline.clear();
		fin.readLine(&textline);
	}

	// now write the output file
	std::string dirname = "../trajectories";
	int exists = open((dirname).c_str(), O_RDONLY);
	if(exists==-1){	//	the directory doesn't exist, must create it
		std::cout << "cannot find directory: " << dirname << "\nExiting...\n";
		exit(1);
//		mkdir(dirname.c_str(), O_RDWR|S_IRWXU|S_IRWXG|S_IRWXO);
	}
	close(exists);
	std::string file_basename = dirname+"/" + basename(argv[1]);
	std::string filename = file_basename;

	int counter = 0;
	exists = open((filename+".trj").c_str(), O_RDONLY);
	while(exists > -1){
		close(exists);
		counter++;
		exists = open((filename + "_" + stringify(counter) + ".trj").c_str(), O_RDONLY);
	}
	filename = (counter>0 ? file_basename + "_" + stringify(counter)+".trj" : file_basename+".trj");

	std::ofstream fout(filename.c_str(), std::ios_base::trunc);
	if (!fout.is_open()){
		std::cout << "Error in creating the output file, aborting...\n";
		exit(1);
	}
	std::cout << "Saving output in " << filename <<"\n";

	// write datas on the file
	for(unsigned int p=0; p<poses.size(); p++){
		RobotPosition * rp = &(poses[p]);
		fout << rp->x() << " " << rp->y() << " " << rp->theta();
		for(unsigned int l=0; l<landmarks.size(); l++){
			fout << " " << computeAngle(&(landmarks[l]), rp);
		}
		fout << "\n";
	}

}
