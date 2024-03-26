% moveToCan
% Part a

%First, Ensure the robot and the world are at the starting position
%Reset the claw to fully open
ResetClaw;
%Reset the objects in the world
resetWorld;
% Return the robot to original position (home position)
goHome('qr');

%Initialize ROS action Client
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','object'); 

%Initialize ROS action Goal
trajGoal = rosmessage(trajAct);

%Load the Robot UR5e into Matlab 
UR5e = loadrobot("UniversalUR5e",DataFormat="row");

%Create the Subscriber to configure the robot in matlab to current Gazebo
%configuration
jointSub = rossubscriber("/joint_states");
jointStateMsg = jointSub.LatestMessage;

%Adjust the forward kinematics to match the URDF model in Gazebo
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

%Create the numerical IK solver for the UR5e
ik = inverseKinematics("RigidBodyTree",UR5e);

%Set the weights
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];

%Receiver the latest joint angles of the robot
jointStateMsg = receive(jointSub,3);

%Set the initial guess to the homeconfiguation (all joint angles to 0)
initialIKGuess = homeConfiguration(UR5e);

% Reconfigure the order of the joint angles to match matlab format
initialIKGuess(1) = jointStateMsg.Position(4); 
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);

%Input the desired end effector pose
%NOTE: X-axis and Y-axis in Gazebo are switched.
gripperX = -.04;
gripperY = 0.8;
gripperZ = 0.13;

%Create array for required Translation and Rotation
gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation = [-pi/2 -pi 0]; %[Z Y Z] radians

%This was our first attempt to get the transformation from the Gazebo
%origin and the Matlab origin
%We hope to use it later

%tftree = rostf('DataFormat','struct')
%tform1 = getTransform(tftree,'world','tool0',rostime('now'), 'Timeout', 2);

%Input the Translation and Rotation arrays to make a homogeneous Transform
tform = eul2tform(gripperRotation);
tform(1:3,4) = gripperTranslation';

%Compute the IKs of the desired end position
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);

%Configure the IKs to the ROS configuration
UR5econfig = [configSoln(3) configSoln(2) configSoln(1) configSoln(4) configSoln(5) configSoln(6)];

%Use the packign function to fill the names and positions into
%the Traj Goal message
trajGoal = packTrajGoal(UR5econfig,trajGoal);

%Send the created goal message to the action server
sendGoal(trajAct,trajGoal);

%Wait for the robot to reach its end position
pause (10)

%Output the End Effector Pose
EndEffectorPose1 = [gripperX, gripperY, gripperZ, gripperRotation(1,1),gripperRotation(1,2),gripperRotation(1,3)]


%APPENDIX - Utility Functions
%These are the packing functions for the gripgoal and the Trajectorygoal
function gripGoal=packGripGoal(pos,gripGoal)
    jointWaypointTimes = 0.1;                                                 % Duration of trajectory
    jointWaypoints = [pos]';                                                % Set position of way points
    numJoints = size(jointWaypoints,1);                                     % Only 1 joint for gripper

    % Joint Names --> gripGoal.Trajectory
    gripGoal.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};

    % Goal Tolerance: set type, name, and pos/vel/acc tolerance
    gripGoal.GoalTolerance = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');

    gripGoal.GoalTolerance.Name = gripGoal.Trajectory.JointNames{1};
    gripGoal.GoalTolerance.Position = 0;
    gripGoal.GoalTolerance.Velocity = 0.1;
    gripGoal.GoalTolerance.Acceleration = 0.1;
    
    % Create waypoint as a ROS trajectory point type. Time for position will be set to TimeFromStart and then just position via JointWaypoints. t 
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat', 'struct');

    trajPts.TimeFromStart = rosduration(jointWaypointTimes,'DataFormat', 'struct');
    trajPts.Positions = jointWaypoints;

    % Zero out everything else
    trajPts.Velocities      = zeros(size(jointWaypoints));
    trajPts.Accelerations   = zeros(size(jointWaypoints));
    trajPts.Effort          = zeros(size(jointWaypoints));
    
    % Copy trajPts --> gripGoal.Trajectory.Points
    gripGoal.Trajectory.Points = trajPts;
end

function trajGoal = packTrajGoal(config,trajGoal)
    jointWaypointTimes = 3;
    jointWaypoints = config';
    numJoints = size(jointWaypoints,1);
    
    trajGoal.Trajectory.JointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
  
    % for idx = 1:numJoints
    % 
    %     % trajGoal.GoalTolerance(idx) = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');
    %     % trajGoal.GoalTolerance(idx).Name = trajGoal.Trajectory.JointNames{idx};
    %     % trajGoal.GoalTolerance(idx).Position = 0;
    %     % trajGoal.GoalTolerance(idx).Velocity = 0;
    %     % trajGoal.GoalTolerance(idx).Acceleration = 0;
    % 
    % end
    
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    trajPts.TimeFromStart = rosduration(jointWaypointTimes);
    trajPts.Positions = jointWaypoints;
    trajPts.Velocities = zeros(size(jointWaypoints));
    trajPts.Accelerations = zeros(size(jointWaypoints));
    trajPts.Effort = zeros(size(jointWaypoints));
    
    trajGoal.Trajectory.Points = trajPts;
end
