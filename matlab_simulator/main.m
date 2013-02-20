% Dopo aver lanciato il main, premere un tasto qualunque per iniziare la
% simulazione. Usare W,S,A,D per spostarsi, X per produrre il file
% odometrico.
%
% Si ricorda il file odometrico � in formato:
% odomPose <ID> noisedX noisedY noisedTheta trueX trueY trueTheta
% bearing bearing1 ... bearingN <IDb1> ... <IDbN>

% Map Properties
xSize = 250;
ySize = 250;
nLandmark = 20;

%Robot Properties
sensorMaxDist=100;

% Simulator Properties
fileOut='outOdometry';


clc
disp('*****************************************************************');
disp('Use Keys: W,S,A,D to move, then press X to produce odometry file');
disp('*****************************************************************');
robLandmarks(xSize,ySize,nLandmark,sensorMaxDist,fileOut);