bearingslam
===========

slam - data association algorithm for bearing only sensor equipped robot.

Required libraries are:
• Eigen
• G2O
• SFML

There are two main programs:

>>>MAPSMERGER<<<
Launch it with "MapsMerger <mapfile1> <mapfile2>"
Mapfiles simply list <x,y> couples, identifying points coordinates
MAPSMERGER will merge them taking into account that their scale is approximatively the same.

>>>LANDMARKSESTIMATOR<<<
Launch it with "LandmarksEstimator <trajectoryfile>".
In order to generate a valid trajectory file for this program, you can use my other project "bearingsimulator".
During the program execution, every pression of the "Enter" key will move the trajectory a step forward.