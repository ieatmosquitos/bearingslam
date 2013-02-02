/*
 * MapsMerger
 *
 * Long story short:	this program takes two maps and tries to merge them.
 *
 * Details:		The algorithm used is the "RANSAC".
 *
 * Future works:	We want to implement a function that "intelligently" chooses which models to try (maybe pruning odd ones)
 *
 * Usage: MapsMerger <filename1> <filename2>
 *
 * Input format:	The input files are supposed to be formatted as "a line per point of interest".
 * 					Example:	142.138 315.862
 * 								608.138 259.862
 * 								407.138 235.862
 * 								149.138 71.8618
 * 								442.138 70.8618
 * 					The first element of the couple is considered to be the "x" value, while the second is the "y" (would you say?!?)
 * 					In the showed images, the y component is considered to increase "going down", so the <0,0> point will be top left corner.
 */

#include "tools.cpp"
#include "graphics_stuff.cpp"
#include "FileReader.cpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DISTANCE_THRESHOLD_DIVIDENDUM	100	// this affects the radius for detecting coincidences


// reads the content of the given map file, and stores the relative map in the given vector
// each line of the file must be composed of two components (namely, x and y),
// lines with too few or too many components WILL BE IGNORED.
void readMapFile(std::string filename,	std::vector<Eigen::Vector3d> * positions) {
	FileReader fr(filename);

	std::vector<std::string> textline;
	fr.readLine(&textline);
	while (fr.good()) {
		if (textline.size() == 2) {
			Eigen::Vector3d v;
			v[0] = atof(textline[0].c_str());
			v[1] = atof(textline[1].c_str());
			v[2] = 1; // this is for getting coordinates in homogeneous form

			positions->push_back(v);
		}
		textline.clear();
		fr.readLine(&textline);
	}
}

// loads the input map files (filename1 and filename2) in the two vectors (positions1 and positions2).
// returns FALSE if the files failed to load, TRUE otherwise.
bool loadMaps(char* filename1, char* filename2,
		std::vector<Eigen::Vector3d> * positions1,
		std::vector<Eigen::Vector3d> * positions2) {
	std::ifstream * f1 = new std::ifstream(filename1);
	std::ifstream * f2 = new std::ifstream(filename2);

	if (!f1->is_open()) {
		//		std::cout << "error opening file << " << filename1 << "\nExiting...\n";
		return false;
	}
	if (!f2->is_open()) {
		//		std::cout << "error opening file << " << filename2 << "\nExiting...\n";
		return false;
	}

	readMapFile(std::string(filename1), positions1);
	readMapFile(std::string(filename2), positions2);

	return true;
}

// v2traslMatrix returns the translation matrix related to the input vector
Eigen::Matrix3d v2traslMatrix(Eigen::Vector3d v) {
	Eigen::Matrix3d ret;
	ret << 1, 0, v[0], 0, 1, v[1], 0, 0, 1;
	return ret;
}

// compute the transformation matrix for moving points i1 and i2 in j1 and j2
Eigen::Matrix3d computeTransformation(Eigen::Vector3d i1, Eigen::Vector3d i2,
		Eigen::Vector3d j1, Eigen::Vector3d j2) {
	// compute translation
	double translation[2];
	translation[0] = j1[0] - i1[0];
	translation[1] = j1[1] - i1[1];

	// compute rotation
	double angle1 = atan2(i2[1] - i1[1], i2[0] - i1[0]);
	double angle2 = atan2(j2[1] - j1[1], j2[0] - j1[0]);
	double rotation = computeAnglesDifference(angle2, angle1);

	// compute scale
	double scale = sqrt(pow(i2[0] - i1[0], 2) + pow(i2[1] - i1[1], 2))
									/ sqrt(pow(j2[0] - j1[0], 2) + pow(j2[1] - j1[1], 2));

	double sin_theta = sin(rotation);
	double cos_theta = cos(rotation);

	cos_theta = (floorf(cos_theta * 1000 + 0.5) / 1000);
	sin_theta = (floorf(sin_theta * 1000 + 0.5) / 1000);

	Eigen::Matrix3d ret;
	ret <<	cos_theta,	-sin_theta,	translation[0],
			sin_theta,	cos_theta,	translation[1],
			0,			0,			scale;

	return ret;
}

// computes the "score" associated to the chosen model
unsigned int compute_score(std::vector<Eigen::Vector3d> * map1,
		std::vector<Eigen::Vector3d> * map2, Eigen::Matrix3d transf,
		Eigen::Vector3d base_vector, double distance_threshold) {
	unsigned int score = 0;

	for (unsigned int i = 0; i < map1->size(); i++) { // for every element of the first map
		Eigen::Vector3d compare_me = v2traslMatrix(base_vector) * transf
				* v2traslMatrix(-base_vector) * (*map1)[i]; // translate-rotate-scale the point

		for (unsigned int j = 0; j < map2->size(); j++) { // look for a "friend" in the other map
			double distance = sqrt(
					pow((*map2)[j][0] - compare_me[0], 2)
					+ pow((*map2)[j][1] - compare_me[1], 2));
			if (distance < distance_threshold) {
				score++;
				// TODO -- maybe break here? What if there are two or more points within range?
			}
		}
	}

	return score;
}

// this decides whether the input choise is plausible or not
bool approveModel(Eigen::Matrix3d * m){
	// Must be the same size
	if(((*m)(2,2) > 1.05) || ((*m)(2,2) < 0.95)){
		return false;
	}

	return true;
}

// finds the transformation matrix with the highest score, stores it in "best_transformation", and returns the index of the first vector of the couple chosen in the first map (the point around which the rotation is performed)
unsigned int find_best_transformation(std::vector<Eigen::Vector3d> * map1,
		std::vector<Eigen::Vector3d> * map2,
		Eigen::Matrix3d * best_transformation, double distance_threshold) {
	// variables for storing the best results
	unsigned int best_points[4]; //	<i1, i2, j1, j2>
	unsigned int most_friends = 0;

	unsigned int evaluated_models = 0;

	// i1 and i2 are indices for map1.
	// j1 and j2 are indices for map2
	for (unsigned int i1 = 0; i1 < map1->size() - 1; i1++) {
		for (unsigned int i2 = i1 + 1; i2 < map1->size(); i2++) {
			// for every couple of points in the first map

			for (unsigned int j1 = 0; j1 < map2->size() - 1; j1++) {
				for (unsigned int j2 = 0; j2 < map2->size(); j2++) {
					// for every couple of points in the second map
					if (j2 == j1)
						continue; // must be a couple!

					Eigen::Matrix3d transf = computeTransformation((*map1)[i1],
							(*map1)[i2], (*map2)[j1], (*map2)[j2]);

					// do checks on the scale
					if(!approveModel(&transf)){
						continue;
					}

					// apply the scale factor
					transf(0,0) = transf(0,0) / transf(2,2);
					transf(0,1) = transf(0,1) / transf(2,2);
					transf(1,0) = transf(1,0) / transf(2,2);
					transf(1,1) = transf(1,1) / transf(2,2);
					transf(2,2) = 1;

					unsigned int friends = compute_score(map1, map2, transf,
							(*map1)[i1], distance_threshold);
					evaluated_models++;

					if (friends > most_friends) {
						most_friends = friends;
						best_points[0] = i1;
						best_points[1] = i2;
						best_points[2] = j1;
						best_points[3] = j2;
						*best_transformation = transf;
					}
				}
			}
		}
	}
	std::cout << "evaluated models: " << evaluated_models << "\n";
	return best_points[0];
}

// merges map1 and map2 in mergedmap, using the RANSAC algorithm
// returns the transformation matrix chosen
unsigned int mergemaps(std::vector<Eigen::Vector3d> * map1,
		std::vector<Eigen::Vector3d> * map2,
		std::vector<Eigen::Vector3d> * mergedmap, Eigen::Matrix3d * transf, double distance_threshold) {

	unsigned int v1_index = find_best_transformation(map1, map2, transf, distance_threshold);

	std::vector<Eigen::Vector3d> tmp;

	for (unsigned int i = 0; i < map1->size(); i++) { // put in tmp all the (transformed) points from map1
		tmp.push_back(
				v2traslMatrix((*map1)[v1_index]) * (*transf) * v2traslMatrix(-(*map1)[v1_index])
				* (*map1)[i]);
	}

	for (unsigned int j = 0; j < map2->size(); j++) { // put in tmp all the (transformed) points from map2
		tmp.push_back((*map2)[j]);
	}

	std::vector<unsigned int> occurrences; // used to keep track of how many points have collapsed in one

	for (unsigned int k = 0; k < tmp.size(); k++) { // collapse points close to each other
		bool collapsed = false;
		for (unsigned int check = 0; check < mergedmap->size(); check++) { // check the already inserted points
			double distance = sqrt(
					pow((*mergedmap)[check][0] - tmp[k][0], 2)
					+ pow((*mergedmap)[check][1] - tmp[k][1], 2));
			if (distance < distance_threshold) { //	must collapse
				collapsed = true;
				double xpos = ((*mergedmap)[check][0] * occurrences[check]
				                                                    + tmp[k][0]) / (occurrences[check] + 1);
				double ypos = ((*mergedmap)[check][1] * occurrences[check]
				                                                    + tmp[k][1]) / (occurrences[check] + 1);
				occurrences[check] = occurrences[check] + 1;
				(*mergedmap)[check][0] = xpos;
				(*mergedmap)[check][1] = ypos;
			}
		}
		if (!collapsed) { // it's a new point
			(*mergedmap).push_back(tmp[k]);
			occurrences.push_back(1);
		}
	}

	return v1_index;
}

// returns an image showing the positions of the points on the map
cv::Mat * drawPOI(std::vector<Eigen::Vector3d> * map) {
	// choose dimentions of the map
	int columns = 0;
	int rows = 0;
	for (unsigned int i = 0; i < map->size(); i++) {
		if ((*map)[i][0] > columns)
			columns = (*map)[i][0];
		if ((*map)[i][1] > rows)
			rows = (*map)[i][1];
	}

	// increase the paint size
	columns = 1.2 * columns;
	rows = 1.2 * rows;

	// create the image
	cv::Mat * image;

	if ((rows > 0) && (columns > 0)) {
		image = new cv::Mat(rows, columns, CV_8UC3, cv::Scalar(255, 255, 200));
	} else {
		*image = cv::imread("images/err_image.png", 0);
		return image;
	}

	int cross_size = MIN(columns, rows) / 40;

	// draw the POIs
	for (unsigned int i = 0; i < map->size(); i++) { // for every POI
		Eigen::Vector3d draw_here = (*map)[i];
		//		draw_here[1] = rows - draw_here[1];
		drawCross(image, draw_here, cross_size,4);
	}

	return image;

}

double computeDistanceTrigger(std::vector<Eigen::Vector3d> * map1,
		std::vector<Eigen::Vector3d> * map2) {
	double x1max = 0;
	double y1max = 0;
	double x2max = 0;
	double y2max = 0;

	// find the maximum value for x and y in both maps
	for (unsigned int i = 0; i < map1->size(); i++) {
		Eigen::Vector3d v = (*map1)[i];
		if (v[0] > x1max)
			x1max = v[0];
		if (v[1] > y1max)
			y1max = v[1];
	}
	for (unsigned int i = 0; i < map2->size(); i++) {
		Eigen::Vector3d v = (*map2)[i];
		if (v[0] > x2max)
			x2max = v[0];
		if (v[1] > y2max)
			y2max = v[1];
	}

	// now choose "the smallest amongst the biggest"
	double dist = x1max;
	if (y1max < dist)
		dist = y1max;
	if (x2max < dist)
		dist = x2max;
	if (y2max < dist)
		dist = y2max;

	return dist / DISTANCE_THRESHOLD_DIVIDENDUM;
}

// creates an image showing the two map images next to each other, and makes lines between correspondences
cv::Mat* createAssociationsImage(cv::Mat * img1, cv::Mat * img2,
		std::vector<Eigen::Vector3d> * map1,
		std::vector<Eigen::Vector3d> * map2, Eigen::Matrix3d * transf, double distance_threshold, unsigned int offset_index) {
	int rows = MAX(img1->rows, img2->rows);
	int columns = img1->cols + img2->cols;

	cv::Mat * associations_image = new cv::Mat(rows, columns, CV_8UC3,
			cv::Scalar(255, 255, 200));

	cv::Mat subimg1 = (*associations_image)(cv::Range(0, img1->rows),
			cv::Range(0, img1->cols));
	cv::Mat subimg2 = (*associations_image)(cv::Range(0, img2->rows),
			cv::Range(img1->cols, img1->cols + img2->cols));
	for (int i = 0; i < subimg1.rows; i++) {
		for (int j = 0; j < subimg1.cols; j++) {
			cv::Vec3b value = img1->at<cv::Vec3b>(i, j);
			subimg1.at<cv::Vec3b>(i, j) = value;
		}
	}
	for (int i = 0; i < subimg2.rows; i++) {
		for (int j = 0; j < subimg2.cols; j++) {
			cv::Vec3b value = img2->at<cv::Vec3b>(i, j);
			subimg2.at<cv::Vec3b>(i, j) = value;
		}
	}

	for (unsigned int i1 = 0; i1 < map1->size(); i1++) {
		Eigen::Vector3d v = v2traslMatrix((*map1)[offset_index]) * (*transf)
													* v2traslMatrix(-(*map1)[offset_index]) * (*map1)[i1];
		for (unsigned int i2 = 0; i2 < map2->size(); i2++) {
			double distance = sqrt(
					pow((*map2)[i2][0] - v[0], 2)
					+ pow((*map2)[i2][1] - v[1], 2));
			if (distance < distance_threshold) {
				Eigen::Vector3d vt = v;
				vt[0] = vt[0] + img1->cols;
				drawLine(associations_image, (*map1)[i1], vt, 2);
			}
		}
	}
	for(int r = 0; r<associations_image->rows; r++){
		associations_image->at<cv::Vec3b>(r,img1->cols)[0] = 0;
		associations_image->at<cv::Vec3b>(r,img1->cols)[1] = 0;
		associations_image->at<cv::Vec3b>(r,img1->cols)[2] = 0;
	}
	return associations_image;
}

// main method
int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage: MapMerger <filename_1> <filename_2>\n";
		exit(1);
	}

	std::cout << "----------MapsMerger---------\n";

	// load maps
	std::vector<Eigen::Vector3d> map1;
	std::vector<Eigen::Vector3d> map2;
	if (!loadMaps(argv[1], argv[2], &map1, &map2)) {
		std::cout << "problems loading the map files\nExiting...\n";
		exit(1);
	}

	//	std::cout << "Map1:\n";
	//	for(unsigned int i=0; i<map1.size(); i++){
	//		std::cout << map1[i][0] << " " << map1[i][1] << "\n";
	//	}
	std::cout << "Maps loaded\n";

	double distance_threshold = computeDistanceTrigger(&map1, &map2);

	std::cout << "distance threshold computed: " << distance_threshold << "\n";

	std::vector<Eigen::Vector3d> totalmap;

	Eigen::Matrix3d transformation;
	unsigned int offset_index = mergemaps(&map1, &map2, &totalmap, &transformation, distance_threshold);

	std::cout << "resulting POIs are in:\n";
	for (unsigned int i = 0; i < totalmap.size(); i++) {
		std::cout << totalmap[i][0] << " " << totalmap[i][1] << "\n";
	}

	// show images

	cvStartWindowThread();
	const char * map1_window_name = "MAP1";
	const char * map2_window_name = "MAP2";
	const char * totalmap_window_name = "TOTALMAP";
	const char * associations_window_name = "ASSOCIATIONS";
	cv::namedWindow(map1_window_name, CV_WINDOW_NORMAL);
	cv::namedWindow(map2_window_name, CV_WINDOW_NORMAL);
	cv::namedWindow(totalmap_window_name, CV_WINDOW_NORMAL);
	cv::namedWindow(associations_window_name, CV_WINDOW_NORMAL);

	cv::Mat * map1_image = drawPOI(&map1);
	cv::Mat * map2_image = drawPOI(&map2);
	cv::Mat * totalmap_image = drawPOI(&totalmap);
	cv::Mat * associations_image = createAssociationsImage(map1_image, map2_image,
			&map1, &map2, &transformation, distance_threshold, offset_index);

	cv::imshow(map1_window_name, *map1_image);
	cv::imshow(map2_window_name, *map2_image);
	cv::imshow(totalmap_window_name, *totalmap_image);
	cv::imshow(associations_window_name, *associations_image);

	std::cout << "Displaying results, press ESC or Q to quit";

	int button = cvWaitKey(0);
	while((button!=27) && ((char)button!='q')){
		button = cvWaitKey(0);
	}

	//	delete transformation;
	//	delete map1_image;
	//	delete map2_image;
	//	delete totalmap_image;
	//	delete associations_image;

	exit(0);
}
