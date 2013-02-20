classdef robLandmarks < handle

    properties
        
        xSize;
        ySize;
        nLandmark;
        landmarks;
        
        handleF;
        
        robotConfiguration; % 2D Homogeneous matrices rapresenting the (real) evolution (integration) of the robot-configuration in the world
        NoisedRobotConfiguration; % 2D Homogeneous matrices rapresenting the noised evolution (integration) of the robot-configuration in the world
        
        traceSizeEE=6;
        traceSizeBase=6;
        
        robVert = cell(3);
        
        h1;
        h2;
        h3;        
        
        noisH1;
        noisH2;
        noisH3;        
        
        AllBearingSeenFromRobot;     
        NoisedAllBearingSeenFromRobot;
        
        %% Noise options
        minimumNoiseLinearSigma = 0.02; % 0.02 minimum sigma for error of sensors (linear).         
        minimumNoiseAngularSigma = 0.02; % 0.02 minimum sigma for error of sensors (angular).
        errorIncreaseWithVelocity = 1; % If 1, when velocity increase you have an encreasing error component.
        
        %%
        minDistanzaPerOdometria; % Minima distanza tra due configurazioni successive affichè si aggiunga una rilevazione odometrica
        AllOdometry;
        NoisedAllOdometry;
        landMarksDistances;
        sensorMaxDistance; % massima distanza landmark
        fileOut;
        nPointsCirc = 36;
        
        robotSize;
        odomGraphFrameSize; % Dimensione assi sistemi di riferimento lasciati come traccia
        
        circleHand;

        
    end
    
    methods
        function obj = robLandmarks(xSize,ySize,nLandmark,sensorMaxDist,fileOut)
            close all
           
            obj.fileOut=fileOut;
            obj.sensorMaxDistance=sensorMaxDist;
            obj.AllBearingSeenFromRobot=cell(1);
            obj.NoisedAllBearingSeenFromRobot=cell(1);
            
            obj.robotConfiguration = cell(1);
            obj.NoisedRobotConfiguration = cell(1);
            
            obj.AllOdometry=cell(1);
            obj.NoisedAllOdometry=cell(1);            
            obj.landMarksDistances = cell(1);            
            
            obj.xSize=xSize;
            obj.ySize=ySize;
            obj.odomGraphFrameSize = obj.xSize/40;
            obj.minDistanzaPerOdometria = obj.xSize/20;
            obj.robotSize = obj.xSize/60;            
            obj.nLandmark=nLandmark;            
            obj.handleF = figure('name','Use Keys: W,S,A,D to move, then press X to produce out file'); 
            
            obj.init();
            
            obj.mainMap();

        end
        
        
        function init(obj)            
            
            % Generate random landmarks
            obj.landmarks = genLandmarks(obj.nLandmark, obj.xSize ,obj.ySize);
            
            % Place robot in the middle of the map, with angle 0
         
            obj.robotConfiguration{1} = valuesToHomog(obj.xSize/2,obj.ySize/2,0);        
            obj.NoisedRobotConfiguration{1} = valuesToHomog(obj.xSize/2,obj.ySize/2,0);        
            
            % Vertices composing robot. They are thinked ordered in circular sequence, when robot is in
            % 0,0, at 0 bearing angle.
            
            obj.robVert{1} = [-obj.robotSize,obj.robotSize,1].';
            obj.robVert{2} = [obj.robotSize*4,0,1].';
            obj.robVert{3} = [-obj.robotSize,-obj.robotSize,1].';

        end            
        
        
        function mainMap(obj)
            disp('Press any key to start the simulation.');
            pause;            
                        
            figure(obj.handleF);
            hold on

            set(obj.handleF,'windowkeypressfcn','set(gcbf,''Userdata'',get(gcbf,''CurrentCharacter''))') ;
            set(obj.handleF,'windowkeyreleasefcn','set(gcbf,''Userdata'','''')') ;
            
            axis equal
            scrsz = get(0,'ScreenSize');
            rect = [0, 0, scrsz(3)/2, scrsz(4)];
            set(obj.handleF,'Position',rect);
            grid off          
                        
            % Draw Landmarks            
            for i = 1 : obj.nLandmark
                plot(obj.landmarks(i,1),obj.landmarks(i,2),'-.r*');
                text(obj.landmarks(i,1),obj.landmarks(i,2),num2str(i));
            end
            
            
            % Calculate vertex of the robot
            
            v1 = obj.robotConfiguration{1}*obj.robVert{1};
            v2 = obj.robotConfiguration{1}*obj.robVert{2};
            v3 = obj.robotConfiguration{1}*obj.robVert{3};
            
            % Graphics
            
                        
            % Robot Real position model
            obj.h1 = line([v1(1),v2(1)],[v1(2),v2(2)],'color','r','EraseMode','normal','linewidth',3);
            obj.h2 = line([v2(1),v3(1)],[v2(2),v3(2)],'color','r','EraseMode','normal','linewidth',3);
            obj.h3 = line([v3(1),v1(1)],[v3(2),v1(2)],'color','r','EraseMode','normal','linewidth',3);
            
            % Robot Noised position model
            obj.noisH1 = line([v1(1),v2(1)],[v1(2),v2(2)],'color','b','EraseMode','normal','linewidth',1);
            obj.noisH2 = line([v2(1),v3(1)],[v2(2),v3(2)],'color','b','EraseMode','normal','linewidth',1);
            obj.noisH3 = line([v3(1),v1(1)],[v3(2),v1(2)],'color','b','EraseMode','normal','linewidth',1);
                       
            % tN = text(-obj.sizeX/2,-obj.sizeY/2,'0');
      
            obj.circleHand = circleHandles([obj.robotConfiguration{1}(1,3),obj.robotConfiguration{1}(2,3)],obj.sensorMaxDistance,obj.nPointsCirc);            
            
            advanceVelocity = 1;
            angularVelocity = 0;
            odoCount = 1;
            timeCount = 1;
            
            while true                
                pause(0.01);              
                
                key = get(obj.handleF,'userdata') ;
                
                minAngularVelocity=pi/128; % Velocità angolare minima (in valore assoluto) che viene applicata in caso di comando di sterzo
                maxAbsAngular = pi/32; % Velocità angolare massima (in valore assoluto) che viene applicata in caso di comando di sterzo
                incrementoVelocitaAngolare = 1.2; % coefficiente > 1 per il quale viene moltiplicata l'attuale velocità angolare in caso di comando di sterzo con stesso segno (modella un incremento esponenziale nella sterzata)
                incrementoVelocitaLineare = 0.1; % Quantità che viene sommata all'attuale velocità lineare in caso di comando di velocità (o sottratta in caso di frenata)
                inerziaSterzo = 0.2; % Quantità (in valore assoluto) che viene sottratta alla attuale velocità angolare in caso di comando di sterzo di segno opposto (modella l'inerzia nello sterzo)
                decadimentoVelocitaAngolare = 1.2; % Coefficiente > 1 per il quale viene divisa l'attuale velocità angolare in assenza di comandi di sterzo
                decadimentoVelocitaLineare = 1.02; % Coefficiente > 1 per il quale viene divisa l'attuale velocità lineare in assenza di comandi sulla velocità lineare (modella un attrito)                
                
                if ~isempty(key),
                    
                    if key =='a' % left
                        if norm(angularVelocity) < minAngularVelocity
                            angularVelocity = minAngularVelocity;
                        end

                        if angularVelocity < 0
                            angularVelocity = angularVelocity  + inerziaSterzo;
                            
                        else
                            angularVelocity = angularVelocity  * incrementoVelocitaAngolare;
                        end
                       
                        if angularVelocity >= maxAbsAngular
                            angularVelocity = maxAbsAngular;
                        end
                        
                        advanceVelocity = advanceVelocity / decadimentoVelocitaLineare;
                    end
                    
                    
                    if key =='d' % right
                        if norm(angularVelocity) < minAngularVelocity
                            angularVelocity= -minAngularVelocity;
                        end
                        
                        if angularVelocity > 0
                            angularVelocity = angularVelocity  -inerziaSterzo;    
                        
                        else
                            angularVelocity = angularVelocity  * incrementoVelocitaAngolare;
                        end                       
                        
                        
                        if angularVelocity <= -maxAbsAngular
                            angularVelocity = -maxAbsAngular;
                        end
                        
                        advanceVelocity = advanceVelocity / decadimentoVelocitaLineare;
     
                    end
                    
                    
                    if key =='w' % increase lin velocity

                        angularVelocity=angularVelocity/decadimentoVelocitaAngolare;
                        advanceVelocity = advanceVelocity+incrementoVelocitaLineare;
                        
                    end
                    
                    if key =='s' % dec lin velocity

                        angularVelocity=angularVelocity/decadimentoVelocitaAngolare;
                        advanceVelocity = advanceVelocity-incrementoVelocitaLineare;
                        
                    end
                    
                    if key =='x' % QUIT
                        disp('Creating output file...');
                        obj.createOdometryFile();
                        break;
                        exit;
                    end
                    
                else
                    advanceVelocity = advanceVelocity / decadimentoVelocitaLineare;
                    angularVelocity= angularVelocity / decadimentoVelocitaAngolare;
                    
                end
                
                % Impose velocity to robot state, and save it in next
                % Config.
                % Then upgrade odometry data structures (integration), but
                % only if distance with respect previous state is enougth
                % far.
                
                                
                % Calculate and update next configuration (noised and unnoised) depending on velocities
                    
                    % Sigma is minimumNoiseSigma + 10% of velocity. So sigma is dependent on
                    % velocity, reflecting the fact that we have higher uncertain
                    % for bigger signals.
                    
                    % Compute sigma
                    if obj.errorIncreaseWithVelocity
                    advanceErrSigma = obj.minimumNoiseLinearSigma + norm(advanceVelocity) *(10/100);
                    angErrSigma = obj.minimumNoiseAngularSigma + norm(angularVelocity) *(10/100);
                    else
                        advanceErrSigma = obj.minimumNoiseLinearSigma;
                        angErrSigma = obj.minimumNoiseAngularSigma;
                    end
                    
                    % Compute error
                    advanceErr = normrnd(0,advanceErrSigma,1,1);
                    angErr = normrnd(0,angErrSigma,1,1);
                    
                    % Compute signals
                    NoisedAdvanceVelocity = advanceVelocity + advanceErr;
                    NoisedAngularVelocity = angularVelocity + angErr;
                    
                    % Compute next configuration matrix and homog. transform matrix (real and noised)
                    [nextConfig, homogVelocity] = advanceRobot( obj.robotConfiguration{timeCount}, advanceVelocity, angularVelocity );
                    obj.robotConfiguration{timeCount+1} = nextConfig;
                    
                    [NoisedNextConfig, NoisedHomogVelocity] = advanceRobot(obj.NoisedRobotConfiguration{timeCount},NoisedAdvanceVelocity,NoisedAngularVelocity);
                    obj.NoisedRobotConfiguration{timeCount+1} = NoisedNextConfig;

                
                % Odometry data structures is upgraded only if distance
                % between (noised) actual configuration and last config
                % where we took an odomeotry measurement, is bigger than a
                % certain amount (or if we are a the first iteration).
                
                if odoCount <= 1
                    configDistances = inf;
                else
                    configDistances = homogDistance(NoisedNextConfig,lastNoisedConfigWithOdometry);
                end
                
                if configDistances > obj.minDistanzaPerOdometria
		    % Upgrade data structures
                    obj.AllOdometry{odoCount} = homogVelocity;
                    obj.NoisedAllOdometry{odoCount} = NoisedHomogVelocity;
                    
                    % Sense landmarks (with respect to the real position of the
                    % robot). Upgrade distance vector too.
                    [bear,dist] = obj.senseLandmark(obj.robotConfiguration{timeCount});
                    obj.AllBearingSeenFromRobot{odoCount} = bear.';
                    obj.landMarksDistances{odoCount} = dist.';  
                    
                    lastConfigWithOdometry = obj.robotConfiguration{timeCount};
                    lastNoisedConfigWithOdometry = obj.NoisedRobotConfiguration{timeCount};
                    
                    % Add to graphic current position of robot the previous
                    % position where odometry was taken
                                  
                    % REAL
                    lastOdoO = lastConfigWithOdometry * [0 0 1]';
                    lastOdoX = lastConfigWithOdometry * [obj.odomGraphFrameSize 0 1]';
                    lastOdoY = lastConfigWithOdometry * [0 obj.odomGraphFrameSize 1]';
                    line([lastOdoO(1),lastOdoX(1)],[lastOdoO(2),lastOdoX(2)],'color','r','EraseMode','normal','linewidth',1);
                    line([lastOdoO(1),lastOdoY(1)],[lastOdoO(2),lastOdoY(2)],'color','r','EraseMode','normal','linewidth',1);
                    
                    
                    % NOISED                   
                    lastOdoO = lastNoisedConfigWithOdometry * [0 0 1]';
                    lastOdoX = lastNoisedConfigWithOdometry * [obj.odomGraphFrameSize 0 1]';
                    lastOdoY = lastNoisedConfigWithOdometry * [0 obj.odomGraphFrameSize 1]';
                    line([lastOdoO(1),lastOdoX(1)],[lastOdoO(2),lastOdoX(2)],'color','b','EraseMode','normal','linewidth',1);
                    line([lastOdoO(1),lastOdoY(1)],[lastOdoO(2),lastOdoY(2)],'color','b','EraseMode','normal','linewidth',1);
                    
                    odoCount = odoCount+1;
                else                    
                
                end
                
                timeCount = timeCount + 1; 
                
                
                % Graphics
                
                % Redraw robot graph model
                v1 = obj.robotConfiguration{timeCount}*obj.robVert{1};
                v2 = obj.robotConfiguration{timeCount}*obj.robVert{2};
                v3 = obj.robotConfiguration{timeCount}*obj.robVert{3};
                set(obj.h1,'xdata', [v1(1),v2(1)],'ydata',[v1(2),v2(2)]);
                set(obj.h2,'xdata', [v2(1),v3(1)],'ydata',[v2(2),v3(2)]);
                set(obj.h3,'xdata', [v3(1),v1(1)],'ydata',[v3(2),v1(2)]);

                nv1 = obj.NoisedRobotConfiguration{timeCount}*obj.robVert{1};
                nv2 = obj.NoisedRobotConfiguration{timeCount}*obj.robVert{2};
                nv3 = obj.NoisedRobotConfiguration{timeCount}*obj.robVert{3};
                
                set(obj.noisH1,'xdata', [nv1(1),nv2(1)],'ydata',[nv1(2),nv2(2)]);
                set(obj.noisH2,'xdata', [nv2(1),nv3(1)],'ydata',[nv2(2),nv3(2)]);
                set(obj.noisH3,'xdata', [nv3(1),nv1(1)],'ydata',[nv3(2),nv1(2)]);                
                
                % Add to graphic current position of robot (real and
                % noised)
%                 
%                 minRap = homogZToValues(obj.robotConfiguration{timeCount});
%                 plot(minRap(1),minRap(2),'color','r');
% 
%                 NoisedMinRap = homogZToValues(obj.NoisedRobotConfiguration{timeCount});
%                 plot(NoisedMinRap(1),NoisedMinRap(2),'color','b');
                
                circleHandlesMove(obj.circleHand,obj.robotConfiguration{timeCount}(1:2,3),obj.sensorMaxDistance,obj.nPointsCirc)

                drawnow; 
                
            end
        end        
        
        function  [bearingSeenFromRobot,landMarkDistances] = senseLandmark(obj,robotConfig)
            
            out = homogZToValues(robotConfig);
            robotPosition = out(1:2);
            robotAngle=out(3);
                        
            for i=1:obj.nLandmark
                displacSeenFromRobot = obj.landmarks(i,:).'-robotPosition(:);
                landAngle = atan2(displacSeenFromRobot(2),displacSeenFromRobot(1));
                a = landAngle - robotAngle;
                bearingSeenFromRobot(i) = mod(a,2*pi); % REM OPPURE MOD?!?!
                landMarkDistances(i) = norm(displacSeenFromRobot);
            end
            
        end
        
        function createOdometryFile(obj)
            nBlocs = size(obj.AllOdometry,2); 
            nRows=0;
            
            % In realtime was applied time to the odometry. Now we apply offline noise to the sensors.
            % I.e. we apply Normal Gaussian Noise to the bearing readings.
            bearingGamma = 0.1;
            
            obj.NoisedAllBearingSeenFromRobot = applyNoiseData(obj.AllBearingSeenFromRobot,0,bearingGamma,2*pi);            
            hTab = '\t';
            
            
            for i=1:nBlocs
                
                nRows=nRows+1;
                % odomPose, <ID>, noisedX noisedY noisedTheta, trueX trueY trueTheta
                fileRow{nRows} = strcat('odomPose',hTab,num2str(i),hTab,num2str(homogZToValues(obj.NoisedAllOdometry{i}).'),hTab,num2str(homogZToValues(obj.AllOdometry{i}).'));
                 
                nRows=nRows+1;
                
                % Select from the current landmark vectors (noised and unnoised) only the measurements that
                % are below a certain distance, reflecting that the
                % sensor's range is not infinite.
                
                % Sort the bearing, reflecting the fact that laser sensor
                % start scanning from its poining.
                % Is included also the vector of the indexing of landmark
                % (#id1 ... idn)
                [noisedSortedBearings,ind] = sort(obj.NoisedAllBearingSeenFromRobot{i});
                realSortedBearings = obj.AllBearingSeenFromRobot{i}(ind);                
                distanceOrdered = obj.landMarksDistances{i}(ind);                
                
                nearLandmarkSortedInd = find((distanceOrdered <= obj.sensorMaxDistance));
                
                noisedBearingsInRadius = noisedSortedBearings(ind(nearLandmarkSortedInd));
                realBearingsInRadius = realSortedBearings(ind(nearLandmarkSortedInd));
                
                nearIndices = ind(nearLandmarkSortedInd);
                
                fileRow{nRows} = strcat('bearing',hTab,num2str(i),hTab,num2str(noisedBearingsInRadius.'),hTab,num2str(realBearingsInRadius.'),hTab,num2str(nearIndices.'));
                
            end            
            
            fOut = fopen(obj.fileOut,'w');
            
            % Create file
            for i=1:nRows

                fprintf(fOut,'%s\r\n',sprintf(fileRow{i}));

            end
            outStr = strcat('Produced output file:',obj.fileOut,'.');            
            disp(outStr);
            
        end
        
    end
    
end

