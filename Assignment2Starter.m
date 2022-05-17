classdef Assignment2Starter < handle
    properties (Constant)
        WATER_LOCATION = [-1,-2.3,1.2];
        SKIM_MILK_LOCATION = [-1,-2.7,1.2];
        REGULAR_MILK_LOCATION = [-1,-3,1.2];
        ENGLISH_BREAKFAST_LOCATION = [-1,-1.7,1.2];
        GREEN_TEA_LOCATION = [-0.7,-1.7,1.2];
        LEMON_GINGER_TEA_LOCATION = [-0.4,-1.7,1.2];
        SPOON_LOCATION = [-0.45 ,-2.2,1.3]
        BARRIER_HEIGHT_MIN = 1;
        BARRIER_HEIGHT_MAX = 2.5;
        CUP_TOTAL = 3;
        HAND_RADII = [0.14,0.2,0.08];
    end
    properties
        L = SingleInstance.Logger;                                          % Logger

        debug;                                                              % Turn off for demo
        h;                                                                  % Emergency Stop handle
        
        robot;                                                              % Dobot Magician  
        calcDobot;                                                          % Calc Dobot - 3DOF dobot used for calculation purposes
        qz = [0,0,0,0];

        % Interactive objects
        cups;                                                               % Array of cups
        coasters;                                                           % Array of coasters
        teaBags;
        sugarcubes;
        spoon;
        sprayBottle;
        sideBarrier;
        frontBarrier;
        warningsign;
        hand;

        orderCount;

        radii;
        centerPoint;
        lightCurtainPoints;
        lightCurtain_h;

    end
    methods
        %% Assignment2Starter - Constructor
        function self = Assignment2Starter(debug)
            close all

            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Instantiated']};

            if nargin < 1
                self.debug = true;
            else
                self.debug = debug;
            end

            self.orderCount = 1;
            SetUpEnvironment(self);

            PlaceCollidableItem(self,[-1,-2,1]);

            %view([80 -70 50]); % Show full kitchen
            %pause;
            SetFigureView(); % Zoom in for clarity 
        end

        %% MakeTea - Make tea using the robot
        function MakeTea(self, teaType, milkType, sugarQuantity)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            disp("Tea order placed. Making tea...");

            if self.orderCount > self.CUP_TOTAL
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'No empty cups remaining. Cannot make more tea.']};
                disp('No empty cups remaining. Cannot make more tea, please reset.');
                return;
            end

            self.lightCurtain_h = InitialiseLightCurtain(self);
            RaiseBarriers(self);      
            GetCup(self, [1,deg2rad(135),deg2rad(45),deg2rad(45)], -pi, transl(self.WATER_LOCATION));            % Pickup cup and place under the water dispenser
            DispenseLiquid(self, self.WATER_LOCATION, self.qz, 'b', 'water');    % remove TODO [-1,-2.35,1.3]                        
            GetTeaBag(self, self.qz, teaType);
            GetSugar(self, self.qz, sugarQuantity);
            GetMilk(self, self.qz, milkType);
            FindCoaster(self);
            StirTea(self);                                                  % Uses RMRC
            ResetPosition(self);

            while self.h %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
            
            LowerBarriers(self);
            delete(self.lightCurtain_h);
            self.lightCurtainPoints = [];

            self.orderCount = self.orderCount + 1;
            disp(['Order count = ', num2str(self.orderCount)]);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Order count = ',num2str(self.orderCount)]};

            disp('MakeTea: Complete');
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Complete']};
        end

        %% InitialiseRobot
        function InitialiseRobot(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'InitialiseRobot: Called'};
            self.robot = Dobot(false);
            self.robot.model.base = self.robot.model.base * transl(-0.7,-3.3,1.08) * trotx(pi/2);
            q = self.robot.model.getpos();
            self.robot.model.animate(q);
        end

        %% InitialiseCalcRobot
        function InitialiseCalcRobot(self)
            self.L.mlog = {self.L.DEBUG, mfilename('class'),'InitialiseCalcRobot: Called'};

            self.calcDobot = ThreeLinkDobot(false);
            self.calcDobot.model.base = self.calcDobot.model.base *  transl(-0.7,-3.3,1.08) * trotx(pi/2);
        end

        %% InitialiseEllipsoids
        function InitialiseEllipsoids(self)
            visualise = false;                                              % Set to true if wanting to visualise ellipsoids

            self.L.mlog = {self.L.DEBUG,mfilename('class'),'InitialiseEllipsoids: Called'};
            self.centerPoint = [0,0,0];
            self.radii{1} = [0.1,0.1,0.1];
            self.radii{2} = [0.1,0.15,0.1];
            self.radii{3} = [0.1,0.15,0.1];
            self.radii{4} = [0.1,0.08,0.05];
            self.radii{5} = [0.1,0.08,0.05];
            self.radii{6} = [0.07,0.1,0.1];
            self.radii{7} = [0.03,0.03,0.03];  
            
            if self.debug && visualise
                for i = 1:self.robot.model.n+1                              % robot links + base
                    [X,Y,Z] = ellipsoid(self.centerPoint(1), self.centerPoint(2), self.centerPoint(3), self.radii{i}(1), self.radii{i}(2), self.radii{i}(3));
                    self.robot.model.points{i} = [X(:),Y(:),Z(:)];
                    warning off
                    self.robot.model.faces{i} = delaunay(self.robot.model.points{i});    
                    warning on;
            
                    self.robot.model.plot3d([0,0,pi/4,pi/4,0,0],'noarrow','workspace',self.robot.workspace);
                end
        
            self.robot.model.plot3d([0,0,pi/4,pi/4,0,0],'noarrow','workspace',self.robot.workspace);
            end 
        end

        %% SetUpEnvironment
        function SetUpEnvironment(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'SetUpEnvironment: Called'};
            disp('Initialising the environment...')
            
            hold on
            % Surrounding Surfaces
            surf([-1.6,-1.6;4,4],[-4,4.5;-4,4.5],[0.01,0.01;0.01,0.01],'CData',imread('tiles.jpg'),'FaceColor','texturemap'); %Floor
            %surf([-4,-4;-4,-4],[-4,4;-4,4],[0,0;2.7,2.7],'CData',flip(imread('tiles.jpg')),'FaceColor','texturemap'); %Back Wall
            %surf([-3,1.8;-3,1.8],[-1.8,-1.8;-1.8,-1.8],[0,0;2.7,2.7],'CData',flip(imread('tiles.jpg')),'FaceColor','texturemap'); %Side Wall

            %Kitchen
            PlaceObject('KitchenBenchWide.ply', [0,0,0]); % Dimensions L(y):7 W(x):1 H(z):1
            PlaceObject('UpperCabinet1.ply', [-0.75,2.7,1.75]); % Dimensions L(y):1 W(x):0.75 H(z):0.75
            PlaceObject('UpperCabinet2.ply', [-0.75,1.7,1.75]); % Dimensions L(y):1 W(x):0.75 H(z):0.75
            PlaceObject('Fridge.ply', [0,3.22,0]); % Dimensions L(y):1.25 W(x):1 H(z):2.25
            
            % Safety Features
            PlaceObject('LightCurtainRear.ply', [-1.45,-1.45,1.04]);
            PlaceObject('LightCurtainFront.ply', [-0.05,-1.45,1.04]);
            PlaceObject('ESBwall.ply', [-0.3,-3.8,0.8]);
            PlaceObject('FE.ply', [-0.5,-3.9,0.39]);
            %Glass barrier - Begins in lowered position
            self.frontBarrier = surf([-0.1,-0.1;-0.1,-0.1],[-3.7,-3.7;-1.45,-1.45],[self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN;self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.3,'EdgeColor','none');
            self.sideBarrier = surf([-1.5,-1.5;-0.1,-0.1],[-3.7,-3.7;-3.7,-3.7],[self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN;self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.3,'EdgeColor','none');

            PlaceObject('hotwaterdispenser.ply', self.WATER_LOCATION); % Set origin at the tap

            PlaceObject('milkdispenserV2.ply', self.SKIM_MILK_LOCATION); % Set origin at the tap
            PlaceObject('milkdispenserV2.ply', self.REGULAR_MILK_LOCATION); % Set origin at the tap

            PlaceObject('teaContainer_EnglishBreakfast.ply',self.ENGLISH_BREAKFAST_LOCATION);
            PlaceObject('teaContainer_Green.ply',self.GREEN_TEA_LOCATION);
            PlaceObject('teaContainer_LemonAndGinger.ply',self.LEMON_GINGER_TEA_LOCATION);

            PlaceObject('sugarcontainer.ply',[-0.45 ,-2.2,1.04]);
        
            for i = 1:self.CUP_TOTAL
                self.cups{i} = MoveableObject('cup.ply');
                self.coasters{i} = MoveableObject('coaster.ply');
            end

            self.coasters{1}.Move(transl(-0.38,-2.5,1.04));
            self.coasters{2}.Move(transl(-0.38,-2.7,1.04));
            self.coasters{3}.Move(transl(-0.38,-2.9,1.04));

            self.cups{1}.Move(transl(-0.9,-3.5,1.12));
            self.cups{2}.Move(transl(-0.7,-3.5,1.12));
            self.cups{3}.Move(transl(-0.5,-3.5,1.12));

            self.cups{1}.Move(transl(-0.55,-3.1,1.12));
            self.cups{2}.Move(transl(-0.55,-3.27,1.12));
            self.cups{3}.Move(transl(-0.55,-3.44,1.12));

            self.spoon = MoveableObject('spoon.ply');
            self.spoon.Move(transl(self.SPOON_LOCATION));

            % Fill sugar container - These are for display only
            PlaceObject('sugarcube.ply',[-0.45,-2.2,1.05]);
            PlaceObject('sugarcube.ply',[-0.47,-2.21,1.05]);
            PlaceObject('sugarcube.ply',[-0.44,-2.24,1.1]);
            PlaceObject('sugarcube.ply',[-0.49,-2.23,1.13]);
            PlaceObject('sugarcube.ply',[-0.41,-2.21,1.14]);
            PlaceObject('sugarcube.ply',[-0.45,-2.25,1.16]);

            InitialiseCalcRobot(self);
            %figure(1) % remove later
            InitialiseRobot(self);
            InitialiseEllipsoids(self);
          
            axis equal
            camlight
        end

        %% PlaceCollidableItem
        function PlaceCollidableItem(self, location)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'PlaceCollidableItem: Called'};
            try delete(self.sprayBottle.mesh); end
            self.sprayBottle = MoveableObject('sprayBottle.ply');
            self.sprayBottle.Move(transl(location));

            if self.debug
                itemPoints = self.sprayBottle.tVertices;
                item_h = plot3(itemPoints(:,1),itemPoints(:,2),itemPoints(:,3),'b.');
            end
        end
        
        %% SimulateWarningSign
        function SimulateWarningSign(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'SimulateWarningSign: Called'};
            self.warningsign = MoveableObject('warningsign.ply'); 
            self.warningsign.Move(transl(-1,-2.3,1.2));                     % Warning sign simulated at hot water dispenser
        end

        %% LowerBarriers
        function LowerBarriers(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'LowerBarriers: Called'};
            if self.frontBarrier.ZData(1,2) && self.sideBarrier.ZData(1,2) == self.BARRIER_HEIGHT_MAX
                for i = self.BARRIER_HEIGHT_MAX:-0.01:self.BARRIER_HEIGHT_MIN
                    self.frontBarrier.ZData(:,2) = i;
                    self.sideBarrier.ZData(:,2) = i;
                    pause(0.03)
                end
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Barrier height is now:', num2str(self.BARRIER_HEIGHT_MIN)]};
            else 
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Barrier is already is lowered position']};
            end
        end

        %% RaiseBarriers
        function RaiseBarriers(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'RaiseBarriers: Called'};
            if self.frontBarrier.ZData(1,2) == self.BARRIER_HEIGHT_MIN && self.sideBarrier.ZData(1,2) == self.BARRIER_HEIGHT_MIN
                for i = self.BARRIER_HEIGHT_MIN:0.01:self.BARRIER_HEIGHT_MAX
                    self.frontBarrier.ZData(:,2) = i;
                    self.sideBarrier.ZData(:,2) = i;
                    pause(0.03)
                end
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Barrier height is now:', num2str(self.BARRIER_HEIGHT_MAX)]};
            else
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Barrier is already is raised position']};
            end
        end

        %% GetCup
        function GetCup(self, qInitial, rotation, location)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'GetCup: Called'};
            while self.h %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end

            GetObject(self, self.cups{self.orderCount}.currentLocation*transl(-0.05,0.05,0.13), qInitial, 50);        
            self.cups{self.orderCount}.goalLocation = location*transl(0.02,0.05,0.1);
            MoveObject(self, self.cups{self.orderCount}, qInitial, 50, rotation);
        end
        
        %% GetTeaBag
        function GetTeaBag(self, qInitial, teaType)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'GetTeaBag: Called'};
            while self.h                                                    % Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
        
            switch teaType
                case 1
                    selectedTeaLocation = transl(self.ENGLISH_BREAKFAST_LOCATION);
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'English Breakfast tea selected!']};
                    disp('English Breakfast tea selected!');
                    waypoint = transl(self.ENGLISH_BREAKFAST_LOCATION(1)+0.1,self.ENGLISH_BREAKFAST_LOCATION(2),self.ENGLISH_BREAKFAST_LOCATION(3)+0.35); %waypoint above tea box

                case 2
                    selectedTeaLocation = transl(self.GREEN_TEA_LOCATION);
                   self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Green tea selected!']};
                    disp('Green tea selected!');
                    waypoint = transl(self.GREEN_TEA_LOCATION(1),self.GREEN_TEA_LOCATION(2),self.GREEN_TEA_LOCATION(3)+0.3); %waypoint above tea box
                case 3
                    selectedTeaLocation = transl(self.LEMON_GINGER_TEA_LOCATION);
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Lemon and Ginger tea selected!']};
                    disp('Lemon and Ginger tea selected!');
                    waypoint = transl(self.LEMON_GINGER_TEA_LOCATION(1)-0.1,self.LEMON_GINGER_TEA_LOCATION(2),self.LEMON_GINGER_TEA_LOCATION(3)+0.35); %waypoint above tea box
                otherwise
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Invalid tea request - Order cancelled']};
                    disp('Invalid tea request. Order has been cancelled, please try again');
                    return
            end
            
            qGoal = qInitial;
            qGoal = self.calcDobot.model.ikcon(selectedTeaLocation,qGoal);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Set of joints to pick up teabag at ',self.L.MatrixToString(selectedTeaLocation),' = ',self.L.MatrixToString(qGoal)]};
            tr = self.calcDobot.model.fkine(qGoal);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Checking result using fkine: ', self.L.MatrixToString(tr)]};
        
            AvoidCollisions(self, self.robot, self.radii, self.centerPoint, qGoal, self.sprayBottle.tVertices, self.L, self.h); % TODO Add a number of tries or do a check first to see if the goal position is in collision and therefore it is impossible
        
            %figure(1);                                                     % plotted figures in other functions cause the teabag to plot on a different plot
            self.teaBags{self.orderCount} = MoveableObject('teabag.ply');   % Instantiate new tea bag
            self.teaBags{self.orderCount}.Move(selectedTeaLocation);
        
            q = self.qz; % ** Change q to suit
            self.teaBags{self.orderCount}.goalLocation = waypoint;
            MoveObject(self,self.teaBags{self.orderCount},q,100,0);% way point before bringing tea bag to cup

            self.teaBags{self.orderCount}.goalLocation = self.cups{self.orderCount}.currentLocation*transl(0.02,0,0.1);
            MoveObject(self, self.teaBags{self.orderCount}, q, 100, 0);     % Pick up teabag and place in cup
        
            UpdateCup(self, 'teaBag');
            delete(self.teaBags{self.orderCount}.mesh);
        end
        
        %% GetSugar
        function GetSugar(self, qInitial, sugarQuantity)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'GetSugar: Called'};
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,num2str(sugarQuantity),' portions of sugar selected!']};
            disp([num2str(sugarQuantity),' portions of sugar selected!']);
        
            if sugarQuantity > 0                                            % Done only if sugar was requested 
                for i=1:sugarQuantity
                    while self.h                                            % Check for emergency stop
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                        pause(1);
                    end
                    self.sugarcubes{i} = MoveableObject('sugarcube.ply'); 

                    self.sugarcubes{i}.Move(transl(-0.45,-2.2,1.05));
                    
                    self.sugarcubes{i}.goalLocation = self.cups{self.orderCount}.currentLocation*transl(0.02,0.04,0.1);

                    waypoint = transl(self.sugarcubes{i}.currentLocation(1),self.sugarcubes{i}.currentLocation(2),self.sugarcubes{i}.currentLocation(3)+0.5);
                    self.sugarcubes{self.orderCount}.goalLocation = waypoint;
                    GetObject(self, self.sugarcubes{i}.goalLocation, qInitial, 50); % go to waypoint before the sugar canister

                    GetObject(self, self.sugarcubes{i}.currentLocation*transl(0,0,0.1), qInitial, 50); % Go to the sugar canister

                    self.sugarcubes{i}.goalLocation = waypoint; %waypoint before bringing sugar to cup
                    MoveObject(self, self.sugarcubes{i}, qInitial, 100, 0);

                    self.sugarcubes{i}.goalLocation = self.cups{self.orderCount}.currentLocation;

                    MoveObject(self, self.sugarcubes{i}, qInitial, 100, 0); % Pick up sugercube and place in cup
                    try delete(self.sugarcubes{i}.mesh); end
                end
            end 
        end
        
        %% GetMilk
        function GetMilk(self, qInitial, milkType)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'GetMilk: Called'};
            while self.h %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
        
            switch milkType
                case 0                                                      % None
                    % Do nothing 
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'No milk selected!']};
                    disp('No milk selected!');
                case 1                                                      % Regular
                    selectedMilkLocation = self.REGULAR_MILK_LOCATION;
                    location = [-1,-3.05,1.3];
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Full Cream milk selected!']};
                    disp('Full Cream milk selected!');
                case 2                                                      % Skim
                    selectedMilkLocation = self.SKIM_MILK_LOCATION;
                    location = [-1,-2.75,1.3];
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Skim milk selected!']};
                    disp('Skim milk selected!');
                otherwise
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Invalid milk request - Order cancelled']};
                    disp('Invalid milk request. Order has been cancelled, please try again');
                    return
            end
        
            if milkType > 0
                GetObject(self, self.cups{self.orderCount}.currentLocation, qInitial, 50); % Get the cup
                self.cups{self.orderCount}.goalLocation = transl(selectedMilkLocation)*transl(0,0,0.1); %0.03 in Z
                MoveObject(self, self.cups{self.orderCount}, qInitial, 100, 0); % Pick up cup and move to under milk dispenser
                DispenseLiquid(self, location, qInitial, 'w', 'milk');
            end 
        end
        
        %% FindCoaster - Pickup cup and place on appropriate available coaster
        function FindCoaster(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'FindCoaster: Called'};
            while self.h                                                    % Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
        
            q = self.qz; % ** Change q to suit 
            GetObject(self, self.cups{self.orderCount}.currentLocation, q, 50);
            self.cups{self.orderCount}.goalLocation = self.coasters{self.orderCount}.currentLocation; % transl(-0.44,-2.5,1.3);
            MoveObject(self, self.cups{self.orderCount}, q, 50, -pi);       % Pick up cup and move to coaster
        end
        
        %% StirTea
        function StirTea(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'StirTea: Called'};
            while self.h                                                    % Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
            GetObject(self, self.spoon.currentLocation, self.qz, 50);
            self.spoon.goalLocation = self.cups{self.orderCount}.currentLocation;
            MoveObject(self, self.spoon, self.qz, 50, 0);
            rmrc = ResolvedMotionRateControl(self.calcDobot,self.robot.model,self.debug,self.h); % TODO add logging in the class
            if self.debug
                line_h1 = plot3(rmrc.x(1,:),rmrc.x(2,:),rmrc.x(3,:),'r.','LineWidth',1);
            end
            x = zeros(3,rmrc.steps/4);
            for i = 1:4:rmrc.steps                                          %Skip some values to increase speed of animation
                while self.h                                                %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                    pause(1);
                end
                newQ = CalcDobotTo6Dof(self,rmrc.qMatrix(i,:),0);
                self.robot.model.animate(newQ);
                modelTr = self.robot.model.fkine(newQ);
                modelTr(1:3,1:3) = eye(3);                                  %Don't change object's rotation
                self.spoon.Move(modelTr);
                x(1:3,i) = modelTr(1:3,4)';                                 % For plotting
                drawnow()
            end
            if self.debug
                line_h2 = plot3(x(1,:),x(2,:),x(3,:),'b.','LineWidth',1);
            end

            self.spoon.goalLocation = transl(self.SPOON_LOCATION);          % Place spooon back in container
            MoveObject(self, self.spoon, self.qz, 50, 0);

            if self.debug
                %try delete(line_h1); end
                %try delete(line_h2); end
            end
        end

        %% ResetPosition
        function ResetPosition(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'ResetPosition: Called'};
            while self.h                                                    % Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
        
            q = [0,0,pi/4,pi/4,0,0];
            steps = 100;
            modelTraj = jtraj(self.robot.model.getpos,q,steps);
        
            for i = 1:steps
                while self.h %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                    pause(1);
                end
                self.robot.model.animate(modelTraj(i,:));
                drawnow()
            end
        end
        
        %% Demonstrate Visual Servoing
        function DemoVisualServoing(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'DemoVisualServoing: Called'};
            RaiseBarriers(self)
            q0 = MoveToTeaArea(self,self.qz,100);
            SimulateWarningSign(self);
            VisualServoingForSign(self,q0,self.warningsign);
        end

        %% Reset Simulation
        function ResetSimulation(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'ResetSimulation: Called'};
            disp('Resetting the environment...')

            for i = 1:self.CUP_TOTAL                                        % Reset cups
                try delete(self.cups{i}.mesh); end
                self.cups{i} = MoveableObject('cup.ply');
            end

            self.cups{1}.Move(transl(-1,-3.6,1.12));
            self.cups{2}.Move(transl(-0.8,-3.6,1.12));
            self.cups{3}.Move(transl(-0.6,-3.6,1.12));

            LowerBarriers(self);                                            % Reset barriers
            ResetPosition(self);                                            % Reset Dobot

            self.orderCount = 1;                                            % Reset order count

            try delete(self.warningsign.mesh); end                          % Delete warning sign
            delete(self.lightCurtain_h);                                    % Stop light curtain
            self.lightCurtainPoints = [];
        end

        %% GetObject - Moves the end effector of the robot model to a set position
        function GetObject(self, location, q, steps)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'GetObject: Called'};
            while self.h %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
        
            q = self.calcDobot.model.ikcon(location,q); 
            tr = self.calcDobot.model.fkine(q);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'GetObject: Set of joints to pick up object at ',self.L.MatrixToString(location),' = ',self.L.MatrixToString(q)]};
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'GetObject: Checking result using fkine: ', self.L.MatrixToString(tr)]};
        
            newQ = CalcDobotTo6Dof(self,q,0);
            tr = self.robot.model.fkine(newQ);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'GetObject: Checking new result using fkine: ', self.L.MatrixToString(tr)]};

            modelTraj = jtraj(self.robot.model.getpos,newQ,steps);
        
            for i = 1:steps
                while self.h                                                % Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                    pause(1);
                end
                self.robot.model.animate(modelTraj(i,:));
                drawnow()
            end
        end
        
        %% MoveObject - Moves the object with the robot end effector to a set location
        function MoveObject(self, object, q, steps, endEffector)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'MoveObject: Called'};
            while self.h                                                    % Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                pause(1);
            end
        
            q = self.calcDobot.model.ikcon(object.goalLocation,q);
            tr = self.calcDobot.model.fkine(q);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'MoveObject: Set of joints to drop off object at ',self.L.MatrixToString(object.goalLocation),' = ',self.L.MatrixToString(q)]};
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'MoveObject: Checking result using fkine: ', self.L.MatrixToString(tr)]};
        
            newQ = CalcDobotTo6Dof(self,q,endEffector);
            tr = self.robot.model.fkine(newQ);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'MoveObject: Checking new result using fkine: ', self.L.MatrixToString(tr)]};
            
            modelTraj = jtraj(self.robot.model.getpos,newQ,steps);
            if endEffector ~= 0
                objectRotationTraj = (1/endEffector):((endEffector-(1/endEffector))/steps):endEffector;
            end
        
            for i = 1:steps
                while self.h                                                % Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                    pause(1);
                end
                self.robot.model.animate(modelTraj(i,:));
                modelTr = self.robot.model.fkine(modelTraj(i,:));
                if endEffector ~= 0 
                    R = trotz(objectRotationTraj(i));
                else 
                    R = object.currentLocation; %trotz(0);
                end
                modelTr(1:3,1:3) = eye(3)*R(1:3,1:3); %Don't change object's rotation
                object.Move(modelTr);
                drawnow()
            end
        end
        
        %% DispenseLiquid
        function DispenseLiquid(self, location, q, colour, teaStage)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'DispenseLiquid: Called'};
            figure(1)    
            GetObject(self, transl(location)*transl(-0.05,-0.05,0.1), q, 50);
            liquid = surf([location(1,1)-0.03,location(1,1)-0.03;location(1,1)-0.03,location(1,1)-0.03],[location(1,2)-0.02,location(1,2)-0.02;location(1,2),location(1,2)],[1,location(1,3)-0.05;1,location(1,3)-0.05],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.9,'EdgeColor',colour);
            pause(3);
            delete(liquid);
            UpdateCup(self, teaStage);
        end
        
        %% UpdateCup - Update the appearance of the cup to match contents
        function UpdateCup(self, teaStage)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'UpdateCup: Called'};
            %figure(1);                                                      %% plotted figures in other functions cause the teabag to plot on a different plot
            currentLocation = self.cups{self.orderCount}.currentLocation;
            try delete(self.cups{self.orderCount}.mesh); end
            switch teaStage
                case 'water'                    
                    self.cups{self.orderCount} = MoveableObject('cupwithwater.ply');
                    self.cups{self.orderCount}.Move(currentLocation);
                case 'teaBag'
                    self.cups{self.orderCount} = MoveableObject('cupwithtea.ply');
                    self.cups{self.orderCount}.Move(currentLocation);
                case 'milk'
                    self.cups{self.orderCount} = MoveableObject('cupwithmilktea.ply');
                    self.cups{self.orderCount}.Move(currentLocation);
            end
        end

        %% CalcDobotTo6Dof - Used to simplfy the calculations on the Dobot due to the hardware limitations of the actual Dobot
        function plotQ = CalcDobotTo6Dof(self, CalcDobotQ, endEffector)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),'CalcDobotTo6Dof: Called'};
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['CalcDobotTo6Dof: CalcDobotQ = ', self.L.MatrixToString(CalcDobotQ)]};

%             % Find the location of the calcDobot
%             trBefore = self.calcDobot.model.fkine(CalcDobotQ);
%             % Modify the end effector location, and add end effector
%             % offsets, end effector height: 0.1, end effector length: 0.06
%             trAfter = trBefore;
%             theta = abs(CalcDobotQ(2));
%             if CalcDobotQ(2) <= 0 && CalcDobotQ(2) >= deg2rad(-135)
%                 trAfter(1,4) = trBefore(1,4) + 0.06 * cos(theta);
%                 trAfter(2,4) = trBefore(2,4) - 0.06 * sin(theta);
%             elseif CalcDobotQ(2) > 0 && CalcDobotQ(2) <= deg2rad(135)
%                 trAfter(1,4) = trBefore(1,4) + 0.06 * cos(theta);
%                 trAfter(2,4) = trBefore(2,4) + 0.06 * sin(theta);
%             else 
%                 disp('ERROR: Q(2) does not meet any conditions');
%             end
%             trAfter(3,4) = trBefore(3,4) + 0.1;
%             % Recalculate joint angles for the offsetted location, ignoring
%             % rotation
%             newQ = self.calcDobot.model.ikine(trAfter,CalcDobotQ,[1,1,1,0,0,0]);
%             %newQ = self.calcDobot.model.ikcon(trAfter,CalcDobotQ);

%             plotQ = zeros(1,6);
%             plotQ(1) = newQ(1);
%             plotQ(2) = newQ(2); 
%             plotQ(3) = newQ(3);
%             plotQ(4) = newQ(4);
%             plotQ(5) = pi/2 - newQ(4) - newQ(3);

            plotQ(1) = CalcDobotQ(1); 
            plotQ(2) = CalcDobotQ(2);
            plotQ(3) = CalcDobotQ(3);
            plotQ(4) = CalcDobotQ(4);
            plotQ(5) = pi/2 - CalcDobotQ(4) - CalcDobotQ(3);
            plotQ(6) = endEffector;
            
%             self.L.mlog = {self.L.DEBUG,mfilename('class'),['CalcDobotTo6Dof: trBefore = ',self.L.MatrixToString(trBefore)]};
%             self.L.mlog = {self.L.DEBUG,mfilename('class'),['CalcDobotTo6Dof: trAfter = ',self.L.MatrixToString(trAfter)]};
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['CalcDobotTo6Dof: plotQ = ', self.L.MatrixToString(plotQ)]};
%             self.L.mlog = {self.L.DEBUG,mfilename('class'),['CalcDobotTo6Dof: newQ = ', self.L.MatrixToString(newQ)]};
%             self.L.mlog = {self.L.DEBUG,mfilename('class'),['CalcDobotTo6Dof: plotQ = ', self.L.MatrixToString(plotQ)]};
         end

        %% Collision Detection - derrived from Lab6Solution
        function result = IsCollision(self,robot, radii, centerPoint, qMatrix, points, L, h)
            L.mlog = {L.DEBUG,mfilename('class'),['IsCollision: ','Called']};
        
            while h %Check for emergency stop
                L.mlog = {L.DEBUG,mfilename('class'),['IsCollision: ','EMERGENCY STOP']};
                pause(1);
            end
        
            result = false;
            for i=1:size(qMatrix,1) %% check 
                newQMatrix(i,:) = CalcDobotTo6Dof(self,qMatrix(i,:),0);
            end
          
            for qIndex = 1:size(newQMatrix,1)      
                tr = GetLinkPoses(newQMatrix(qIndex,:), robot.model);
            
                for i = 3:size(tr,3)                                                % Ignore first and second ellipsoid (base and prismatic link) to reduce calculations 
                    pointsAndOnes = [inv(tr(:,:,i)) * [points,ones(size(points,1),1)]']';
                    updatedPoints = pointsAndOnes(:,1:3);
                    algebraicDist = GetAlgebraicDist(updatedPoints, centerPoint, radii{i});
                    pointsColliding = find(algebraicDist <= 1,1);                       % added the ,1 to improve performance (suggestion from MATLAB)
                    if ~isempty(pointsColliding)
                        result = true;
                        disp(['Collision! Ellipsoid #',num2str(i)]);
                        L.mlog = {L.DEBUG,mfilename('class'),['DetectCollisions: ','Collision detected inside the ',num2str(i),'th ellipsoid']};
                        return;
                    end
                end
            end
        end
            
        %% Collision Avoidance - derrived from Lab6Solution (From current position to goal position)
        function AvoidCollisions(self, robot, radii, centerPoint, qGoal, points, L, h)
            L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','Called']};
            while h %Check for emergency stop
                L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','EMERGENCY STOP']};
                pause(1);
            end
        
            q1 = robot.model.getpos();
            q1 = q1(1,1:4);
            qWaypoints = [q1;qGoal];
            isCollision = true;
            checkedTillWaypoint = 1;
            qMatrix = [];
            while (isCollision)
                startWaypoint = checkedTillWaypoint;
                for i = startWaypoint:size(qWaypoints,1)-1
                    qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
                    if ~IsCollision(self, robot, radii, centerPoint, qMatrixJoin, points, L, h)
                        qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                        size(qMatrix)
                        isCollision = false;
                        checkedTillWaypoint = i+1;
                        % Now try and join to the final goal (qGoal)
                        qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); qGoal],deg2rad(10));
                        if ~IsCollision(self, robot, radii, centerPoint, qMatrixJoin, points, L, h)
                            qMatrix = [qMatrix;qMatrixJoin];
                            % Reached goal without collision, so break out
                            break;
                        end
                    else
                        % Randomly pick a pose that is not in collision
                        qRand = (2 * rand(1,4) - 1) * pi;
                        while (IsCollision(self, robot, radii, centerPoint, qRand, points, L, h) || ~WithinLimits(robot, CalcDobotTo6Dof(self,qRand,0))) %%which robot to use for limits????
                            qRand = (2 * rand(1,4) - 1) * pi;
                        end
                        qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                        isCollision = true;
                        break;
                    end
                end
            end
            for i=1:size(qMatrix,1)
                while h %Check for emergency stop
                    L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','EMERGENCY STOP']};
                    pause(1);
                end
                newQMatrix = CalcDobotTo6Dof(self,qMatrix(i,:),0);
                robot.model.animate(newQMatrix);
                pause(0.05);
            end
        end

        function isCollision = PlaceHand(self, location)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};

            [X,Y,Z] = ellipsoid(self.centerPoint(1), self.centerPoint(2), self.centerPoint(3), self.HAND_RADII(1), self.HAND_RADII(2), self.HAND_RADII(3));
            %ellipsoid_h = surf(X,Y,Z,FaceAlpha=0.1);                       % Plot ellipsoid to visualise 
            self.hand = MoveableObject('hand.ply');
            self.hand.Move(transl(location));
            isCollision = false;
            i = 0.1;
            if ~isempty(self.lightCurtainPoints)
                while ~isCollision
                    location(1,2) = location(1,2)-i;
                    self.hand.Move(transl(location));
                    isCollision = CheckForCollisionInLightCurtain(self);
                    if isCollision
                        self.h = true;
                        disp('Stopping Robot');
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Stopping Robot']};
                        return;
                    end
                    i = i+0.1;
                end
            else 
                isCollision = false;
                self.h = false;
                disp('Light curtain not detected');
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Light curtain not detected']};
            end
        end

        function RemoveHand(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};

            IsCollision = true;
            i = 0.1;
            if ~isempty(self.lightCurtainPoints)
                while IsCollision
                    location = self.hand.currentLocation; 
                    location(2,4) = location(2,4)+i;
                    self.hand.Move(location);
                    isCollision = CheckForCollisionInLightCurtain(self);
                    if ~isCollision
                        self.h = false;
                        disp('Collision with light curtain no longer detected - Continue');
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Collision with light curtain no longer detected - Continue']};
                        delete(self.hand.mesh);
                        return;
                    end
                    i = i+0.1;
                end
            else 
                self.h = false;
                disp('Light curtain not detected');
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Light curtain not detected']};
                delete(self.hand.mesh);
            end
        end

        function isCollision = CheckForCollisionInLightCurtain(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};

            try
                pointsAndOnes = [inv(self.hand.currentLocation) * [self.lightCurtainPoints,ones(size(self.lightCurtainPoints,1),1)]']';
                updatedPoints = pointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedPoints, self.centerPoint, self.HAND_RADII);
                pointsInside = find(algebraicDist <= 1);
                if any(pointsInside)
                    isCollision = true;
                    disp('Collision with light curtain! Stopping Robot');
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Collision detected between hand and light curtain']};
                else
                    isCollision = false;
                end
            catch
                isCollision = false;
            end
        end

        function lightCurtain_h = InitialiseLightCurtain(self)
            [X,Z] = meshgrid(-0.7:0.01:0.7,0:0.07:0.8);
            sizeMat = size(X);
            Y = repmat(0.75,sizeMat(1),sizeMat(2));
            self.lightCurtainPoints = [X(:),Y(:),Z(:)]; % Combine one surface as a point cloud
            self.lightCurtainPoints = self.lightCurtainPoints + repmat([-0.75,-2.2,1.05],size(self.lightCurtainPoints,1),1);
            lightCurtain_h = plot3(self.lightCurtainPoints(:,1),self.lightCurtainPoints(:,2),self.lightCurtainPoints(:,3),'r.');
        end

        %% TestFunction - For testing/debug purposes only
        function TestFunction(self, testOption)
            %clf
            %hold on
            %axis equal

            self.debug = true;
            
            %InitialiseRobot(self);
            %InitialiseCalcRobot(self);

            switch testOption
                case 1                                                      % Test Calculation Robot (3-link) to displayed robot
                    q = [0,0,0,0];
                    location = self.cups{2}.currentLocation;
                    GetObject(self, location, q, 100);
                case 2                                                      %Test Collision Avoidance
                    PlaceCollidableItem(self,[-0.9,-3,1]);
                    itemPoints = self.sprayBottle.tVertices;
                    item_h = plot3(itemPoints(:,1),itemPoints(:,2),itemPoints(:,3),'b.');         
                    InitialiseEllipsoids(self)
                    qGoal = deg2rad([0,-134,45,45,0,0]);
                    AvoidCollisions(self, self.robot, self.radii, self.centerPoint, qGoal, itemPoints, self.L, self.h);
                case 3                                                      % Test RMRC
                    StirTea(self);
                    %RMRC(self.robot.model, self.cups{1}, transl(self.WATER_LOCATION), self.L, self.h, self.debug);
                case 4
                    % @Sam, add your visual servoing code here to test and
                    % call a.TestFunction(4)
                    %visual servoing for sign
                    q0 = MoveToTeaArea(self,self.qz,100);
                    SimulateWarningSign(self);
                    VisualServoingForSign(self,q0,self.warningsign);
                case 5
                    hold on 
                    axis equal
                    lightCurtain_h = InitialiseLightCurtain(self);
                    value = self.PlaceHand([-0.6,-1,1.5]);
            end
        end
    end
end

%% SetFigureView
function SetFigureView()
    view([50 -70 50]);
    axis([ -3, 1, -4, 0, -2, 3]);
    camzoom(2);
end

%% WithinLimits - Function to determine if a set of joint angles is within the robot's joint limits
function result = WithinLimits(robot, q)
    result = true;
    for i=1:robot.model.n
        if ~((q(i) > robot.model.qlim(i,1)) && (q(i) < robot.model.qlim(i,2)))
            result = false;
            return;
        end
    end
end
%% Moves robot to tea area before starting visual servoing
function IdealQs = MoveToTeaArea(self, q, steps)
    self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveToTeaArea: ','Called']};

    %put robot in an area in the coaster area to start visual servoing
    IdealQs = [-0.887,-pi/2,pi/4,pi/4];
%       IdealQs = [0,-pi/2,pi/4,pi/4]; %for finding target

    newQ = CalcDobotTo6Dof(self,IdealQs,0);

    %calcTraj = jtraj(self.calcDobot.model.getpos,q,steps); % Remove TODO
    modelTraj = jtraj(self.robot.model.getpos,newQ,steps);

    for i = 1:steps
        while self.h %Check for emergency stop
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveToCoasterArea: ','EMERGENCY STOP']};
            pause(1);
        end
        %self.calcDobot.model.animate(calcTraj(i,:)); % Remove TODO
        self.robot.model.animate(modelTraj(i,:));
        drawnow()
    end
    
end
%% Visual servoing code to retreat from warning sign
function VisualServoingForSign(self,q0,object) %adapted from Lab 8 visual servoing code
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['VisualServoingForSign: ','Called']};
            while self.h %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),['VisualServoingForSign: ','EMERGENCY STOP']};
                pause(1);
            end

            % Create image target          
            signStar = [  953.0248  898.4115  898.4115  953.0248
                          547.3127  547.3127  601.9260  601.9260]; 

            distancetosetsignpoints = 0.05;

            signpoints = [object.currentLocation(1,4),object.currentLocation(1,4),object.currentLocation(1,4),object.currentLocation(1,4);
                            object.currentLocation(2,4)+distancetosetsignpoints,object.currentLocation(2,4)-distancetosetsignpoints,object.currentLocation(2,4)-distancetosetsignpoints,object.currentLocation(2,4)+distancetosetsignpoints;
                            object.currentLocation(3,4)+distancetosetsignpoints,object.currentLocation(3,4)+distancetosetsignpoints,object.currentLocation(3,4)-distancetosetsignpoints,object.currentLocation(3,4)-distancetosetsignpoints];
            % uncomment below to see sign points plotted on warning sign in workspace
%             figure(1)
%             plot_sphere(signpoints,0.02,'b')

            %Add the camera (specs similar to Lab 8 visual servoing code)
            %added 'sensor' to increase FOV
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', 'DOBOTcam','sensor',0.5);
            % frame rate
            fps = 100;

            %Define values
            %gain of the controler
            lambda = 0.6;
            %depth of the IBVS
            depth = mean (signpoints(1,:));

            f2 = figure(2);
            self.calcDobot.model.plot(q0,'workspace',self.calcDobot.workspace,'nojoints','scale',0.1,'noarrow');
            Tc0 = self.calcDobot.model.fkine(q0);

            %plot camera
            cam.T = Tc0;
            cam.plot_camera('Tcam',Tc0 ,'label','scale',0.05);
            lighting gouraud
            light
            
            %Display in image view
            %Project points to the image %uncomment below when deciding what target points to set
%             p = cam.plot(signpoints, 'Tcam', Tc0);

            %camera view and plotting
            cam.clf()
            cam.plot(signStar, '*'); % create the camera view
            cam.hold(true);
            cam.plot(signpoints, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            cam.hold(true);
            cam.plot(signpoints);    % show initial view

            %Initialise display arrays
            vel_p = [];
            uv_p = [];
            history = [];

            %% 1.4 Loop
            % loop of the visual servoing
            ksteps = 0;
             while true
                 while self.h %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),['VisualServoingForSign: ','EMERGENCY STOP']};
                    pause(1);
                end
                    ksteps = ksteps + 1;
                    
                    % compute the view of the camera
                    uv = cam.plot(signpoints);
                    
                    % compute image plane error as a column
                    e = uv-signStar;  % feature error
                    e = e(:);
                    Zest = [];
                    

                    % compute the Jacobian
                    if isempty(depth)
                        % exact depth from simulation (not possible in practice)
                        pt = homtrans(inv(Tcam), signpoints);
                        J = cam.visjac_p(uv, pt(3,:) );
                    elseif ~isempty(Zest)
                        J = cam.visjac_p(uv, Zest);
                    else
                        J = cam.visjac_p(uv, depth );
                    end
            
                    % compute the velocity of camera in camera frame
                    try
                        v = lambda * pinv(J) * e;
                    catch
                        status = -1;
                        return
                    end
                    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            
                    %compute robot's Jacobian and inverse
                    J2 = self.calcDobot.model.jacobn(q0);
                    Jinv = pinv(J2);
                    % get joint velocities
                    qp = Jinv*v;
            
                     
                     %Maximum angular velocity cannot exceed 180 degrees/s
                     ind=find(qp>pi);
                     if ~isempty(ind)
                         qp(ind)=pi;
                     end
                     ind=find(qp<-pi);
                     if ~isempty(ind)
                         qp(ind)=-pi;
                     end
            
                    %Update joints
                    q = q0' + (1/fps)*qp;
                    self.calcDobot.model.animate(q');
                    newQ = CalcDobotTo6Dof(self,q',0);
                    self.robot.model.animate(newQ);
            
                    %Get camera location
                    Tc = self.calcDobot.model.fkine(q);
                    %Tc = Tc*trotx(-pi/2);
                    cam.T = Tc;
                    drawnow
                    
                    % update the history variables
                    hist.uv = uv(:);
                    vel = v;
                    hist.vel = vel;
                    hist.e = e;
                    hist.en = norm(e);
                    hist.jcond = cond(J);
                    hist.Tcam = Tc;
                    hist.vel_p = vel;
                    hist.uv_p = uv;
                    hist.qp = qp;
                    hist.q = q;
            
                    history = [history hist];
            
                     pause(1/fps)
            
                    if ~isempty(500) && (ksteps > 200)
                        break;
                    end
                    
                    %update current joint position
                    q0 = q';                
             end
             
            %% 1.5 Plot results
            if self.debug
                figure()            
                plot_p(history,signStar,cam)
                figure()
                plot_camera(history)
                figure()
                plot_vel(history)
                figure()
                plot_robjointpos(history)
                figure()
                plot_robjointvel(history)
            end
            close(f2);
end

%% plot_ - Functions for plotting visual servoing(From Lab 8 Solution)
function plot_p(history,uv_star,camera)
    %VisualServo.plot_p Plot feature trajectory
    %
    % VS.plot_p() plots the feature values versus time.
    %
    % See also VS.plot_vel, VS.plot_error, VS.plot_camera,
    % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
    
    if isempty(history)
        return
    end
    figure();
    clf
    hold on
    % image plane trajectory
    uv = [history.uv]';
    % result is a vector with row per time step, each row is u1, v1, u2, v2 ...
    for i=1:numcols(uv)/2
        p = uv(:,i*2-1:i*2);    % get data for i'th point
        plot(p(:,1), p(:,2))
    end
    plot_poly( reshape(uv(1,:), 2, []), 'o--');
    uv(end,:)
    if ~isempty(uv_star)
        plot_poly(uv_star, '*:')
    else
        plot_poly( reshape(uv(end,:), 2, []), 'rd--');
    end
    axis([0 camera.npix(1) 0 camera.npix(2)]);
    set(gca, 'Ydir' , 'reverse');
    grid
    xlabel('u (pixels)');
    ylabel('v (pixels)');
    hold off
end
function plot_vel(history)
    %VisualServo.plot_vel Plot camera trajectory
    %
    % VS.plot_vel() plots the camera velocity versus time.
    %
    % See also VS.plot_p, VS.plot_error, VS.plot_camera,
    % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
    if isempty(history)
        return
    end
    clf
    vel = [history.vel]';
    plot(vel(:,1:2), '-')
    hold on
    plot(vel(:,3:4), '--')
    hold off
    ylabel('Cartesian velocity')
    grid
    xlabel('Time')
    xaxis(length(history));
    legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z')
end
function plot_camera(history)
    %VisualServo.plot_camera Plot camera trajectory
    %
    % VS.plot_camera() plots the camera pose versus time.
    %
    % See also VS.plot_p, VS.plot_vel, VS.plot_error,
    % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.

    if isempty(history)
        return
    end
    clf
    % Cartesian camera position vs time
    T = reshape([history.Tcam], 4, 4, []);
    subplot(211)
    plot(transl(T));
    ylabel('camera position')
    grid
    subplot(212)
    plot(tr2rpy(T))
    ylabel('camera orientation')
    grid
    xlabel('Time')
    xaxis(length(history));
    legend('R', 'P', 'Y');
    subplot(211)
    legend('X', 'Y', 'Z');
end
function plot_robjointvel(history)
  
    if isempty(history)
        return
    end
    clf
    vel = [history.qp]';
    plot(vel(:,1:4), '-')
    hold on
    ylabel('Joint velocity')
    grid
    xlabel('Time')
    xaxis(length(history));
    legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_5', '\omega_6')
end
function plot_robjointpos(history)       
    if isempty(history)
        return
    end
    clf
    pos = [history.q]';
    plot(pos(:,1:4), '-')
    hold on
    ylabel('Joint angle')
    grid
    xlabel('Time')
    xaxis(length(history));
    legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6')
 end

%% GetAlgebraicDist - from Lab6Solution
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
%
% *Inputs:*
% _points_ (many*(2||3||6) double) x,y,z cartesian point
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:* 
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
    algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end

%% GetLinkPoses - adapted from Lab5_Solution_Question2and3
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)
    links = robot.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.base;
    
    for i = 1:length(links)
        L = links(1,i);
        
        current_transform = transforms(:,:, i);
        if i == 1
            current_transform = current_transform * trotz(L.theta) * ...
            transl(0,0,q(1,i)) * transl(L.a,0,0) * trotx(L.alpha);
        else
            current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
            transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        end
        transforms(:,:,i + 1) = current_transform;
    end
end

%% InterpolateWaypointRadians - from Lab5_Solution_Question2and3
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
    if nargin < 2
        maxStepRadians = deg2rad(1);
    end
    
    qMatrix = [];
    for i = 1: size(waypointRadians,1)-1
        qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
    end
end

%% FineInterpolation - from Lab5_Solution_Question2and3
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
    if nargin < 3
        maxStepRadians = deg2rad(1);
    end
        
    steps = 2;
    while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
        steps = steps + 1;
    end
    qMatrix = jtraj(q1,q2,steps);
end