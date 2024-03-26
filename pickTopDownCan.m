%pickTopDownCan
%Part c 

%Reset claw to open position
ResetClaw

% Use the moveTopDownCan script to move the gripper to a position to grasp
% can
moveTopDownCan

%Pause before the moving the gripper to be sure it is settled
pause(5)

%Create a client based on the gripper controller action topic
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory')
%Create the gripper goal message                              
gripGoal    = rosmessage(grip_client);

%Input desired gripper pose
gripPos     = 0.23;

%Use the Gripper packing function for to fill the gripper message
gripGoal    = packGripGoal(gripPos,gripGoal);

%Send the created goal message to the action server
sendGoal(grip_client,gripGoal)

%wait for the end effector to reach its final pose
pause(10)
%output final poses
EndEffectorPose3 = [gripperX, gripperY, gripperZ, gripperRotation(1,1), gripperRotation(1,2), gripperRotation(1,3)]