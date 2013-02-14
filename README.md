bearingslam
===========

slam - data association algorithm for bearing only sensor equipped robot.

Required libraries are:
• Eigen
• G2O
• SFML


Compiling (under Linux):
mkdir build
cd build
cmake ..
make

There are two main programs:



>>>MAPSMERGER<<<
Launch it with "MapsMerger <mapfile1> <mapfile2>"
Mapfiles simply list <x,y> couples, identifying points coordinates
MAPSMERGER will merge them taking into account that their scale is approximatively the same.
Example maps in the "maps" directory are not always useful because they refer to a previous version of the program that allowed map scaling.


>>>LANDMARKSESTIMATOR<<<
Launch it with "LandmarksEstimator <trajectoryfile>".
In order to generate a valid trajectory file for this program, you can use my other project "bearingsimulator".
During the program execution, every pression of the "Enter" key will move the trajectory a step forward.
Alternatively, you can press the SPACE BAR to let the algorithm run till the end.
When the association algorithm has reached its end, you can press "R" key to relaunch the algorithm using the poses found as initial guess.
If you want to try it, you can already find some trajectory file in the "trajectories" directory.
