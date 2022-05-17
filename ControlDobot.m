classdef ControlDobot < handle
    % Class used to control the physical Dobot. Sets up a limited
    % simulation and allows for the virtual placing of objects and
    % calculate their movements

    % To be used the dobot must be connected via USB, dobot_magician_driver must be running, and ROS_HOSTNAME
    % set as localhost.
    
    properties(Constant)
        % Set constant end effector properties for use in end effector
        % offsetting when using the real Dobot
        END_EFFECTOR_HEIGHT = 0.1;
        END_EFFECTOR_LENGTH = 0.06;
    end

    properties
    % Robot and associated joint states
%        robot
%        simulatedQ
%        actualQ

    % Movable Objects
 %       cup

    % Subscribers
         safetyStatusSubscriber

    % Publishers and their messages
         safetyStatePublisher
         safetyStateMsg
         targetEndEffectorPub
         targetEndEffectorMsg
         targetJointTrajPub
         targetJointTrajMsg
         toolStatePub
         toolStateMsg
    end

    methods
        % Control Dobot - Initialise the ROS environment (including subs
        % and pubs) and initialise the dobot for calculations
        function obj = ControlDobot()
             rosshutdown;
             rosinit;

            % Setup safety status subscriber
            obj.safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');

            % Initialise the dobot
            [obj.safetyStatePublisher,obj.safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
            obj.safetyStateMsg.Data = 2;
            send(obj.safetyStatePublisher,obj.safetyStateMsg);

            % Setup target joint end effector publisher
            [obj.targetEndEffectorPub,obj.targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
            % Setup target joint state publisher
            [obj.targetJointTrajPub,obj.targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
            % Setup the gripper tool publisher
            [obj.toolStatePub, obj.toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');

        % Set up simulation
            % Setup the robot
            %obj.robot = PhysDobot;
            %obj.simulatedQ = deg2rad([0, 5, 45]);   % Set a sensible initial joint state
            %obj.robot.model.plot(obj.simulatedQ);
            % Setup the cup
            %obj.cup = MoveableObject('cup.ply');
            %obj.cup.Move(transl(0.2,0.2,0));
            
        end

        % Initialise - Used to initialise the physical dobot connected to
        % via ROS
        function [ ] = initialise(obj)
            obj.safetyStateMsg.Data = 2;
            send(obj.safetyStatePublisher,obj.safetyStateMsg);
        end

        % setPose - Sets the simulated pose, and calculates the actual pose
        % taking into account the end effector offset.
            % q - Joint state that is set
            % endEffector - end effector joint state (which isn't typically
            % simulated, just hard coded)
        function [ ] = setPose(obj, q, endEffector)
            % Save the current joint state
            obj.simulatedQ = q;

            % Find the location of the PhysDobot
            trBefore = obj.robot.model.fkine(q);
            % Modify the end effector location, and add end effector
            % offsets, end effector height: 0.1, end effector length: 0.06
            trAfter = trBefore;
            trAfter(1,4) = trBefore(1,4) + obj.END_EFFECTOR_LENGTH * cos(CalcDobotQ(2));
            trAfter(2,4) = trBefore(2,4) + obj.END_EFFECTOR_LENGTH * sin(CalcDobotQ(2));
            trAfter(3,4) = trBefore(3,4) + obj.END_EFFECTOR_HEIGHT;
            % Recalculate joint angles for the offsetted location, ignoring
            % rotation - Possible failure point as ikine does not take into
            % account joint limits, but does allow end effector masking
            newQ = self.calcDobot.model.ikine(trAfter,q,[1,1,1,0,0,0]);

            obj.actualQ = zeros(1,4);
            obj.actualQ(1) = newQ(1); % plotQ(2) = CalcDobotQ(2)
            obj.actualQ(2) = newQ(2); % plotQ(3) = CalcDobotQ(3)
            obj.actualQ(3) = newQ(3); % plotQ(4) = CalcDobotQ(4)
            obj.actualQ(4) = endEffector;
            obj.simulatedQ = q;
        end

        % Used for trajectory calculation, unused as movement was hardcoded
        % for demonstration video
        function GetObject(obj, location, q, steps)
            % Joints to pick
            q = self.calcDobot.model.ikcon(location,q);         
            newQ = CalcDobotTo6Dof(self,q,0);
            modelTraj = jtraj(obj.robot.model.getpos,newQ,steps);
        
            for i = 1:steps
                obj.robot.model.animate(modelTraj(i,:));
                drawnow()
            end
        end

        % Move to given xyz point, upon testing, rotation seems to be
        % non-functional
        function [ ] = moveToPoint(obj, point, rotation)
            obj.targetEndEffectorMsg.Position.X = point(1);
            obj.targetEndEffectorMsg.Position.Y = point(2);
            obj.targetEndEffectorMsg.Position.Z = point(3);
            
            qua = eul2quat(rotation);
            obj.targetEndEffectorMsg.Orientation.W = qua(1);
            obj.targetEndEffectorMsg.Orientation.X = qua(2);
            obj.targetEndEffectorMsg.Orientation.Y = qua(3);
            obj.targetEndEffectorMsg.Orientation.Z = qua(4);
            
            send(obj.targetEndEffectorPub,obj.targetEndEffectorMsg);
        end

        % Use this function to set tool state, vac: 1 - vacuum ON, 0 - vacuum OFF, close:
        % 1 - tool closed, 0 - tool open
        function [ ] = setTool(obj, vac, close)
            obj.toolStateMsg.Data = [vac close];
            send(obj.toolStatePub,obj.toolStateMsg);
        end
    end
end