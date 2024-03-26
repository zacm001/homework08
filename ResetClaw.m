%ResetClaw
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory')
                              
gripGoal    = rosmessage(grip_client);
gripPos     = 0.0;
gripGoal    = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal)
