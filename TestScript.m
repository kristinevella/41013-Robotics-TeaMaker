% %% Script to move the actual Dobot
% % ROS Initialisation
% rosinit;
% %%
% % Setup safety status subscriber. Used to cancel robot connection of
% % robot cannot be moved
% safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
% 
% % Initialise the Dobot
% [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
% safetyStateMsg.Data = 2;
% send(safetyStatePublisher,safetyStateMsg);
% 
% %% Start the target joint state publisher
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% %% Start the target end effector publisher
% [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
% 
% %%
% myRobot = PhysDobot(false)
% view([1 0 0])
% %% Get the simulation to a certain joint angle and create the equivalent joint angle for the actual robot
% q = [-0.3, 0.4, 0.2, 0 ,0];
% 
% %myRobot.model.animate(q);
% actualQ(1) = q(1);
% actualQ(2) = q(2);
% actualQ(3) = q(3) - 0.3; %- pi/2 + q(2);
% actualQ(4) = q(5);
% %% Send a joint target
% %actualQ = [0,0.4,0.3+pi/4,0]; % Remember that the Dobot has 4 joints by default.
% trajectoryPoint.Positions = actualQ;
% targetJointTrajMsg.Points = trajectoryPoint;
% send(targetJointTrajPub,targetJointTrajMsg);

%% Test the class
A = ControlDobot


%% Move to Above Cup
point = [0.2,0.2,0.1];
rotation = [0, 0, 0];
A.moveToPoint(point, rotation);
pause(2)
%% Open Gripper
A.setTool(1,0);

%% Move to Cup
point = [0.2,0.2,0.04];
rotation = [0, 0, 0];
A.moveToPoint(point, rotation);
pause(2)
%% Close Gripper
A.setTool(1,1);

%% Move to Above Cup
point = [0.2,0.2,0.1];
rotation = [0, 0, 0];
A.moveToPoint(point, rotation);
pause(2)
%% Move to Water
point = [0.2,0.0,0.04];
rotation = [0, 0, 0];
A.moveToPoint(point, rotation);
pause(2)
%% Open Gripper
A.setTool(1,0);

%% Move to Above Cup
point = [0.2,0.0,0.1];
rotation = [0, 0, 0];
A.moveToPoint(point, rotation);
pause(2)
%% Push water button
point = [0.25,0.0,0.13];
A.moveToPoint(point, rotation);
pause(0.5);
point = [0.25,0.0,0.1]
A.moveToPoint(point, rotation);
pause(2)
%% Move to Cup
point = [0.25,0.0,0.13];
A.moveToPoint(point, rotation);
pause(0.5);
point = [0.2,0.0,0.1];
A.moveToPoint(point, rotation);
pause(0.5);
point = [0.2,0,0.04];
A.moveToPoint(point, rotation);
pause(2)
%% Close Gripper
A.setTool(1,1);

%% Move to original cup location
point = [0.2,0.2,0.1];
A.moveToPoint(point, rotation);
pause(0.5);
point = [0.2,0.2,0.04];
A.moveToPoint(point, rotation);
pause(5);
A.setTool(1,0);

%% Get teabag
point = [0.2,0.2,0.1];
A.moveToPoint(point, rotation);
pause(2);
point = [0.0, 0.2, 0.1];
A.moveToPoint(point, rotation);
pause(2);
point = [0.0, 0.2, -0.02];
A.moveToPoint(point, rotation);
pause(2)
%% Close Gripper
A.setTool(1,1);

%% Move to cup
point = [0.0, 0.2, 0.1];
A.moveToPoint(point, rotation);

%% Move to centre of cup 
point = [0.215,0.17,0.1];
A.moveToPoint(point, rotation);

%% Lower closer to water
point = [0.215,0.17,0.07];
A.moveToPoint(point, rotation);

%% Drop tea
A.setTool(1,0);
pause(1)
A.setTool(0,0);
