clear all
close all
clc

%% cost map
costmap = helperSLCreateCostmap_Track1();
h = figure;
plot(costmap)

%% Route planner
maxSteeringAngle = 35; % in degrees
vehicleDims = vehicleDimensions(4.5,1.7); 
costmap.CollisionChecker.VehicleDimensions = vehicleDims;

load('plannerpoints')
load('teta')
currentPose =[out(1,1) out(1,2) teta_req(1,1)];
for i = 1:1:length(out)
% if length(teta_req)>length(out)
    StartPose = [out(i,1) out(i,2) teta_req(i,1)];
% else
%     teta_req((length(out)),1)= teta_req(8,1);
% end
end
StartPose = [out(:,1) out(:,2) teta_req(:,1)];
 EndPose =[]
for j = 2:1:length(StartPose)
    EndPose  = [EndPose;[StartPose(j,1) StartPose(j,2) teta_req(j,1)]];  
end
for  k= length(EndPose)+1
    EndPose(k,1) = currentPose(1,1);
    EndPose(k,2) = currentPose(1,2);
    EndPose(k,3) = currentPose(1,3);  
end
  degree = [StartPose(:,3) EndPose(:,3)];
 data = table(StartPose,EndPose,degree);
 RoutePlan = data;
  data = load('routePlanSL4.mat');
  routePlan = data.routePlan %#ok<NOPTS>
 currentPose = [routePlan.StartPose(1,1) routePlan.StartPose(1,2) routePlan.StartPose(1,3)];
hold on
helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose')
legend
for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};

    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

BehavioralPlanner = HelperBehavioralPlanner(routePlan, maxSteeringAngle);

motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000, ...
    'ConnectionDistance',30, 'MinTurningRadius', 5);

goalPose = routePlan{1, 'EndPose'};

refPath = plan(motionPlanner, currentPose, goalPose);

refPath.PathSegments

[transitionPoses, directions] = interpolate(refPath);

% Visualize the planned path
plot(motionPlanner)
hold off
% Specify number of poses to return using a separation of approximately 0.1 m
approxSeparation = 0.1; % meters
%refPath.Length1 = 1
refPath1.Length = 30;
%numSmoothPoses1   = round(refPath1.Length / approxSeparation);
numSmoothPoses   = round(refPath1.Length / approxSeparation);

% Return discretized poses along the smooth path
%[refPoses1, directions1, cumLengths1, curvatures1] = smoothPathSpline(transitionPoses1, directions1, numSmoothPoses1);
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
% Plot the smoothed path
hold on
% hSmoothPath1 = plot(refPoses1(:, 1), refPoses1(:, 2), 'g', 'LineWidth', 2, ...
%     'DisplayName', 'Smoothed Path1');
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), '--k', 'LineWidth', 1, ...
    'DisplayName', 'Smoothed Path2');


%% Velocity profiler
maxSpeed   = 10; % in meters/second
startSpeed = 1; % in meters/second
endSpeed   = 0; % in meters/second

refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);
% figure (2)
% plot(refVelocities,'k', 'LineWidth', 2)
% hold on
% plot (maxSpeed,'g', 'LineWidth', 2)
% hold off
%plotVelocityProfile(cumLengths, refVelocities, maxSpeed)
%% Vehicle control and simulation
%closeFigures;

% Create the vehicle simulator
vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);

% Set the vehicle pose and velocity 
vehicleSim.setVehiclePose(currentPose);
currentVel = 0;
vehicleSim.setVehicleVelocity(currentVel);

% Configure the simulator to show the trajectory
vehicleSim.showTrajectory(true);

% Hide vehicle simulation figure
showFigure(vehicleSim);

% x0 = 59;
% y0 = 180;
% theta0= 90;
pathAnalyzer = HelperPathAnalyzer(refPoses, refVelocities, directions, ...
    'Wheelbase', vehicleDims.Wheelbase)
sampleTime = 0.05;
lonController = HelperLongitudinalController('SampleTime', sampleTime);
controlRate = HelperFixedRate(1/sampleTime); % in Hertz
reachGoal = false;

% 
while ~reachGoal    
    % Find the reference pose on the path and the corresponding velocity
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    % Update driving direction for the simulator
    updateDrivingDirection(vehicleSim, direction);
    
    % Compute steering command
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
   

    
    % Compute acceleration and deceleration commands
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    % Simulate the vehicle using the controller outputs
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    % Check if the vehicle reaches the goal
    reachGoal = helperGoalChecker(goalPose, currentPose, currentVel, endSpeed, direction);
    
    % Wait for fixed-rate execution
    waitfor(controlRate);
    
    % Get current pose and velocity of the vehicle
    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
    
    
    
end
% 
% Show vehicle simulation figure

showFigure(vehicleSim);
% figure(4)
% plot((0:length(Str)-1)*j,Str)
%hold on
%plot(curvatures)
%% Execute a Complete Plan
% Now combine all the previous steps in the planning process and run the
% simulation for the complete route plan. This process involves 
% incorporating the behavioral planner.

% Set the vehicle pose back to the initial starting point
currentPose = [59,180, 90]; % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

% Reset velocity
currentVel  = 0; % meters/second
vehicleSim.setVehicleVelocity(currentVel);
Str=[];
Refpose = [];
currpose =[];
acccmd = [];
currvel = [];
refvel = [];

while ~reachedDestination(BehavioralPlanner)
    
    % Request next maneuver from behavioral layer
    [nextGoal, plannerConfig, speedConfig] = requestManeuver(BehavioralPlanner, ...
        currentPose, currentVel);
    
    % Configure the motion planner
    configurePlanner(motionPlanner, plannerConfig);
    
    % Plan a reference path using RRT* planner to the next goal pose
    refPath = plan(motionPlanner, currentPose, nextGoal);
    
    % Check if the path is valid. If the planner fails to compute a path,
    % or the path is not collision-free because of updates to the map, the
    % system needs to re-plan. This scenario uses a static map, so the path
    % will always be collision-free.
    isReplanNeeded = ~checkPathValidity(refPath, costmap);
    if isReplanNeeded
        warning('Unable to find a valid path. Attempting to re-plan.')
        
        % Request behavioral planner to re-plan
        replanNeeded(BehavioralPlanner);
        continue;
    end
    
    % Retrieve transition poses and directions from the planned path
    [transitionPoses, directions] = interpolate(refPath);
     
    % Smooth the path
    numSmoothPoses   = round(refPath.Length / approxSeparation);
    [refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
    
    % Generate a velocity profile
    refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);
    
    % Configure path analyzer
    pathAnalyzer.RefPoses     = refPoses;
    pathAnalyzer.Directions   = directions;
    pathAnalyzer.VelocityProfile = refVelocities;
    
    % Reset longitudinal controller 
    reset(lonController);
    
    reachGoal = false;
    
    % Execute control loop
    while ~reachGoal  
        % Find the reference pose on the path and the corresponding velocity
        [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
        
        % Update driving direction for the simulator
        updateDrivingDirection(vehicleSim, direction);
        
        % Compute steering command
        steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
            'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
        
        % Compute acceleration and deceleration commands
        lonController.Direction = direction;
        [accelCmd, decelCmd] = lonController(refVel, currentVel);
        
        % Simulate the vehicle using the controller outputs
        drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
        
        % Check if the vehicle reaches the goal
        reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
        
        % Wait for fixed-rate execution
        waitfor(controlRate);
        
        % Get current pose and velocity of the vehicle
        currentPose  = getVehiclePose(vehicleSim);
        currentVel   = getVehicleVelocity(vehicleSim);
        
            Str(end+1)= steeringAngle;
            Refpose (end+1)  = refPose;
            currpose (end+1) = currentPose;
            currvel (end+1) = currentVel ;
            refvel (end+1) = refVel;
            
            
    end
end

% Show vehicle simulation figure
showFigure(vehicleSim);

%load('helperSLCreateUtilityBus.mat');
open_system('RRT_Track1');
set_param('RRT_Track1','SimulationCommand','Update');
open_system('RRT_Track1/Vehicle Controller')
open_system('RRT_Track1/Vehicle Model');
sim('RRT_Track1')