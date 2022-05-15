classdef Assignment2Starter < handle
    properties (Constant)
        WATER_LOCATION = [-1,-2.3,1.2];
        SKIM_MILK_LOCATION = [-1,-2.7,1.2];
        REGULAR_MILK_LOCATION = [-1,-3,1.2];
        ENGLISH_BREAKFAST_LOCATION = [-1,-1.7,1.2];
        GREEN_TEA_LOCATION = [-0.7,-1.7,1.2];
        LEMON_GINGER_TEA_LOCATION = [-0.4,-1.7,1.2];
        BARRIER_HEIGHT_MIN = 1;
        BARRIER_HEIGHT_MAX = 2.5;
        CUP_TOTAL = 3;
    end
    properties
        %Logger
        L = SingleInstance.Logger;

        debug;                                                          % Turn off for demo
        
        %Emergency Stop handle
        h;

        % Dobot Magician
        robot;
        % Calc Dobot - 3DOF dobot used for calculation purposes
        calcDobot;
        qz = [0,0,0,0];

        % Interactive objects
        cups; % Array of cups
        coasters; % Array of coasters
        teaBags;
        sugarcube;
        spoon;
        sprayBottle;
        sideBarrier;
        frontBarrier;
        warningsign;

        orderCount;

        radii;
        centerPoint;

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

            PlaceCollidableItem(self,[-1,-2,1]); %% Do we want this here from the beginning?

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

            RaiseBarriers(self);      
            GetCup(self, self.qz, pi, transl(-1,-2.3,1.28));                % Pickup cup and place under the water dispenser
            DispenseLiquid(self, [-1,-2.35,1.3], self.qz, 'b', 'water');                        
            GetTeaBag(self, self.qz, teaType);
            GetSugar(self, self.qz, sugarQuantity);
            GetMilk(self, self.qz, milkType);
            FindCoaster(self, milkType);                                    % Uses Visual Servoing
            StirTea(self);                                                  % Uses RMRC
            ResetPosition(self);

            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
            
            LowerBarriers(self);

            self.orderCount = self.orderCount + 1;
            disp(['Order count = ', num2str(self.orderCount)]);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Order count = ',num2str(self.orderCount)]};

            disp('MakeTea: Complete');
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Complete']};
        end

        %% InitialiseRobot
        function InitialiseRobot(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['InitialiseRobot: ','Called']};
            self.robot = Dobot(false);
            self.robot.model.base = self.robot.model.base * transl(-0.7,-3.3,1.08) * trotx(pi/2); % Moved implementation of robot location from robot class, to Assignment2Starter
            q = self.robot.model.getpos();
            self.robot.model.animate(q);
        end

        %% InitialiseCalcRobot
        function InitialiseCalcRobot(self)
            self.L.mlog = {self.L.DEBUG, mfilename('class'),['InitialiseCalcRobot: ', 'Called"']};

            self.calcDobot = ThreeLinkDobot(false);
            self.calcDobot.model.base = self.calcDobot.model.base *  transl(-0.7,-3.3,1.08) * trotx(pi/2);
        end

        %% InitialiseEllipsoids
        function InitialiseEllipsoids(self)
            visualise = false;                                              % Set to true if wanting to visualise ellipsoids

            self.L.mlog = {self.L.DEBUG,mfilename('class'),['InitialiseEllipsoids: ','Called']};
            self.centerPoint = [0,0,0];
            self.radii{1} = [0.1,0.1,0.1];
            self.radii{2} = [0.1,0.15,0.1];
            self.radii{3} = [0.1,0.15,0.1];
            self.radii{4} = [0.1,0.08,0.05];
            self.radii{5} = [0.1,0.08,0.05];
            self.radii{6} = [0.07,0.1,0.1];
            self.radii{7} = [0.03,0.03,0.03];  
            
            if self.debug && visualise
                % Visualise ellispoids
                for i = 1:self.robot.model.n+1                                               % robot links + base
                    [X,Y,Z] = ellipsoid(self.centerPoint(1), self.centerPoint(2), self.centerPoint(3), self.radii{i}(1), self.radii{i}(2), self.radii{i}(3));
                    self.robot.model.points{i} = [X(:),Y(:),Z(:)];
                    warning off
                    self.robot.model.faces{i} = delaunay(self.robot.model.points{i});    
                    warning on;
            
                    self.robot.model.plot3d([0,0,pi/4,pi/4,0,0],'noarrow','workspace',self.robot.workspace);
                end
        
            self.robot.model.plot3d([0,0,pi/4,pi/4,0,0],'noarrow','workspace',self.robot.workspace); %TODO Change workspace??
            end 
        end

        %% SetUpEnvironment
        function SetUpEnvironment(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['SetUpEnvironment: ','Called']};
            disp('Initialising the environment...')
            
            hold on
            % Surrounding Surfaces
            surf([-1.6,-1.6;4,4],[-4,4.5;-4,4.5],[0.01,0.01;0.01,0.01],'CData',imread('tiles.jpg'),'FaceColor','texturemap'); %Floor
            %surf([-4,-4;-4,-4],[-4,4;-4,4],[0,0;2.7,2.7],'CData',flip(imread('tiles.jpg')),'FaceColor','texturemap'); %Back Wall
            %surf([-3,1.8;-3,1.8],[-1.8,-1.8;-1.8,-1.8],[0,0;2.7,2.7],'CData',flip(imread('lab2.jpg')),'FaceColor','texturemap'); %Side Wall

            %Kitchen
            PlaceObject('KitchenBenchWide.ply', [0,0,0]); % Dimensions L(y):7 W(x):1 H(z):1
            PlaceObject('UpperCabinet1.ply', [-0.75,2.7,1.75]); % Dimensions L(y):1 W(x):0.75 H(z):0.75
            PlaceObject('UpperCabinet2.ply', [-0.75,1.7,1.75]); % Dimensions L(y):1 W(x):0.75 H(z):0.75
            PlaceObject('Fridge.ply', [0,3.22,0]); % Dimensions L(y):1.25 W(x):1 H(z):2.25
            
            % Safety Features
            PlaceObject('ESBwall.ply', [-0.3,-3.8,0.8]);
            PlaceObject('FE.ply', [-0.5,-3.9,0.39]);
            %Glass barrier - Begins in lowered position
            self.frontBarrier = surf([-0.1,-0.1;-0.1,-0.1],[-3.7,-3.7;-1.3,-1.3],[self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN;self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.3,'EdgeColor','none');
            self.sideBarrier = surf([-1.5,-1.5;-0.1,-0.1],[-3.7,-3.7;-3.7,-3.7],[self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN;self.BARRIER_HEIGHT_MIN,self.BARRIER_HEIGHT_MIN],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.3,'EdgeColor','none');
            % TODO fix location of barrier so it is not on coasters 

            %PlaceObject('SeatedGirl.ply', [-2.5,-1.2,0])

            PlaceObject('hotwaterdispenser.ply', self.WATER_LOCATION); % Set origin at the tap

            PlaceObject('milkdispenserV2.ply', self.SKIM_MILK_LOCATION); % Set origin at the tap
            PlaceObject('milkdispenserV2.ply', self.REGULAR_MILK_LOCATION); % Set origin at the tap

            PlaceObject('teaContainer_EnglishBreakfast.ply',self.ENGLISH_BREAKFAST_LOCATION);
            PlaceObject('teaContainer_Green.ply',self.GREEN_TEA_LOCATION);
            PlaceObject('teaContainer_LemonAndGinger.ply',self.LEMON_GINGER_TEA_LOCATION);

            PlaceObject('sugarcontainer.ply',[-0.45 ,-2.2,1.04]); %% TODO Move (out of reach)
        
            for i = 1:self.CUP_TOTAL
                self.cups{i} = MoveableObject('cup.ply');
                self.coasters{i} = MoveableObject('coaster.ply');
            end


            self.coasters{1}.Move(transl(-0.38,-2.5,1.04));
            self.coasters{2}.Move(transl(-0.38,-2.7,1.04));
            self.coasters{3}.Move(transl(-0.38,-2.9,1.04));

            self.cups{1}.Move(transl(-1,-3.6,1.12));
            self.cups{2}.Move(transl(-0.8,-3.6,1.12));
            self.cups{3}.Move(transl(-0.6,-3.6,1.12));


            self.sugarcube = MoveableObject('sugarcube.ply'); 
            self.sugarcube.Move(transl(-0.45 ,-2.2,1.05));

            InitialiseCalcRobot(self);
            figure(1) % remove later
            InitialiseRobot(self);
            InitialiseEllipsoids(self);
          
            axis equal
            camlight
        end

        %% PlaceCollidableItem
        function PlaceCollidableItem(self, location)
            try delete(self.sprayBottle.mesh); end
%             if isempty(self.sprayBottle)
%                 self.sprayBottle = MoveableObject('sprayBottle.ply'); % Initialise spray bottle
%             end
            self.sprayBottle = MoveableObject('sprayBottle.ply'); % Initialise spray bottle
            self.sprayBottle.Move(transl(location));

            if self.debug
                itemPoints = self.sprayBottle.tVertices;
                item_h = plot3(itemPoints(:,1),itemPoints(:,2),itemPoints(:,3),'b.');
            end
        end
        
        %% SimulateWarningSign
        function SimulateWarningSign(self)
            self.warningsign = MoveableObject('warningsign.ply'); 
            self.warningsign.Move(transl(-1,-2.3,1.2)); %warning sign simulated at hot water dispenser
        end

        %% LowerBarriers
        function LowerBarriers(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
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
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
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
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end

            GetObject(self, self.cups{self.orderCount}.currentLocation, qInitial, 50);        
            self.cups{self.orderCount}.goalLocation = location;
            MoveObject(self, self.cups{self.orderCount}, qInitial, 50, rotation);
        end
        
        %% GetTeaBag
        function GetTeaBag(self, qInitial, teaType)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
        
            switch teaType
                case 1
                    selectedTeaLocation = transl(self.ENGLISH_BREAKFAST_LOCATION);
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'English Breakfast tea selected!']};
                    disp('English Breakfast tea selected!');
                case 2
                    selectedTeaLocation = transl(self.GREEN_TEA_LOCATION);
                   self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Green tea selected!']};
                    disp('Green tea selected!');
                case 3
                    selectedTeaLocation = transl(self.LEMON_GINGER_TEA_LOCATION);
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Lemon and Ginger tea selected!']};
                    disp('Lemon and Ginger tea selected!');
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
        
            AvoidCollisions(self.robot, self.radii, self.centerPoint, qGoal, self.sprayBottle.tVertices, self.L, self.h); % TODO Add a number of tries or do a check first to see if the goal position is in collision and therefore it is impossible
        
            %try delete(self.teaBag); end % remove after testing
            %figure(1);                                                      %% plotted figures in other functions cause the teabag to plot on a different plot
            self.teaBags{self.orderCount} = MoveableObject('teabag.ply');   % Instantiate new tea bag
            self.teaBags{self.orderCount}.Move(selectedTeaLocation);
        
            self.teaBags{self.orderCount}.goalLocation = self.cups{self.orderCount}.currentLocation;
            q = self.qz; % ** Change q to suit
            MoveObject(self, self.teaBags{self.orderCount}, q, 100, 0); % Pick up teabag and place in cup
        
            UpdateCup(self, 'teaBag');
        
            %% TODO what do we do with the tea bag now? does it stay in the cup and follow or do we just delete it?
            self.teaBags{self.orderCount}.Move(transl(0,0,0));
        end
        
        %% GetSugar
        function GetSugar(self, qInitial, sugarQuantity)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end

            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,num2str(sugarQuantity),' portions of sugar selected!']};
            disp([num2str(sugarQuantity),' portions of sugar selected!']);
        
            if sugarQuantity > 0                                        % Done only if sugar was requested 
                for i=1:sugarQuantity
                    GetObject(self, self.sugarcube.currentLocation, qInitial, 50); % Go to the sugar canister
                    self.sugarcube.goalLocation = self.cups{self.orderCount}.currentLocation;
                    MoveObject(self, self.sugarcube, qInitial, 100, 0); % Pick up sugercube and place in cup
            
                    if self.h == true %Check for emergency stop
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                        return
                    end
                end
            end 
        end
        
        %% GetMilk
        function GetMilk(self, qInitial, milkType)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
        
            switch milkType
                case 0                                                  % None
                    % Do nothing 
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'No milk selected!']};
                    disp('No milk selected!');
                case 1                                                  % Regular
                    selectedMilkLocation = [-1,-3.0,1.23]; %self.REGULAR_MILK_LOCATION;
                    location = [-1,-3.05,1.3];
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Full Cream milk selected!']};
                    disp('Full Cream milk selected!');
                case 2                                                  % Skim
                    selectedMilkLocation = [-1,-2.7,1.23]; %self.SKIM_MILK_LOCATION;
                    location = [-1,-2.75,1.3];
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Skim milk selected!']};
                    disp('Skim milk selected!');
                otherwise
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Invalid milk request - Order cancelled']};
                    disp('Invalid milk request. Order has been cancelled, please try again');
                    return
            end
        
            %% Need to add way points so it doesn't go through the containers
            if milkType > 0
                GetObject(self, self.cups{self.orderCount}.currentLocation, qInitial, 50); % Get the cup
                self.cups{self.orderCount}.goalLocation = transl(selectedMilkLocation);
                MoveObject(self, self.cups{self.orderCount}, qInitial, 100, 0); % Pick up cup and move to under milk dispenser
                DispenseLiquid(self, location, qInitial, 'w', 'milk');
            end 
        end
        
        %% FindCoaster - Pickup cup and place on appropriate available coaster (Visual servoing part)
        function FindCoaster(self, milkType)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
        
            q = self.qz; % ** Change q to suit 
            if milkType > 0
                GetObject(self, self.cups{self.orderCount}.currentLocation, q, 50);
            else
                GetObject(self, self.cups{self.orderCount}.currentLocation, q, 50);
            end
            
            %MoveToFindCoaster(self.robot.model, self.cups{self.orderCount}, q, 100, self.L); % TODO Add emergency stop check 
            %MoveToFindCoaster(self.robot.model, self.cups{self.orderCount},q, 100, self.L); % TODO Add emergency stop check 
            %q0 = self.robot.model.getpos(); % ** Change q to suit 
        
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
            
            %Create coaster in workspace %TODO move to setup part of code
            %coaster = [-0.67;-3.6;1.08]; %TODO change later, using current position to make sure dobot can actually reach
        %             coaster = [-0.67,-0.67,-0.77,-0.77;
        %                     -3.6,-3.5,-3.5,-3.6;
        %                      1.04,1.04,1.04,1.04];
        %             figure(1)
        %             plot_sphere(coaster,0.05,'b') 
            %plot_circle(coaster,0.15,'b') %TODO fill colour
            
            %VisualServoing(self.robot.model,q0,coaster); %TODO FIX!!!!(-0.44,-2.5,1.04)
            %pause
            q = self.qz; % ** Change q to suit
            if milkType > 0
                self.cups{self.orderCount}.goalLocation = transl(-0.44,-2.5,1.3); %self.coasters{self.orderCount}.currentLocation;
                MoveObject(self, self.cups{self.orderCount}, q, 50, -pi); % Pick up cup and move to coaster
            else 
                self.cups{self.orderCount}.goalLocation = transl(-0.44,-2.5,1.3); %self.coasters{self.orderCount}.currentLocation;
                MoveObject(self, self.cups{self.orderCount}, q, 50, -pi); % Pick up cup and move to coaster
            end
        end
        
        %% StirTea
        function StirTea(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
            GetObject(self, self.cups{self.orderCount}.currentLocation*transl(0,0,0.2), self.qz, 50);
            rmrc = ResolvedMotionRateControl(self.calcDobot,self.robot.model,self.debug,self.h); % TODO add logging in the class
            if self.debug
                line_h1 = plot3(rmrc.x(1,:),rmrc.x(2,:),rmrc.x(3,:),'r.','LineWidth',1);
            end
            x = zeros(3,rmrc.steps/4);
            for i = 1:4:rmrc.steps %Skip some values to increase speed of animation
                if self.h == true %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),['RMRC: ','EMERGENCY STOP']};
                    return
                end
                newQ = CalcDobotTo6Dof(rmrc.qMatrix(i,:),0);
                self.robot.model.animate(newQ);
                modelTr = self.robot.model.fkine(newQ);
                %modelTr(1:3,1:3) = eye(3); %Don't change object's rotation
                %object.Move(modelTr); %TODO spoon or stiring stick?
                x(1:3,i) = modelTr(1:3,4)'; % for plotting
                drawnow()
            end
            if self.debug
                line_h2 = plot3(x(1,:),x(2,:),x(3,:),'b.','LineWidth',1);
            end
%             pause
%             try delete(line_h1); end
%             try delete(line_h1); end
        end

        %% ResetPosition
        function ResetPosition(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
        
            q = [0,0,pi/4,pi/4,0,0];
            steps = 100;
            modelTraj = jtraj(self.robot.model.getpos,q,steps);
        
            for i = 1:steps
                if self.h == true %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                    return
                end
                self.robot.model.animate(modelTraj(i,:));
                drawnow()
            end
        end
        
        %% GetObject - Moves the end effector of the robot model to a set position
        function GetObject(self, location, q, steps)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['GetObject: ','Called']};
        
            q = self.calcDobot.model.ikcon(location,q); 
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['GetObject: Set of joints to pick up object at ',self.L.MatrixToString(location),' = ',self.L.MatrixToString(q)]};
        
            tr = self.calcDobot.model.fkine(q);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['GetObject: Checking result using fkine: ', self.L.MatrixToString(tr)]};
        
            newQ = CalcDobotTo6Dof(q,0);
        
            %calcTraj = jtraj(self.calcDobot.model.getpos,q,steps); % Remove TODO
            modelTraj = jtraj(self.robot.model.getpos,newQ,steps);
        
            for i = 1:steps
                if self.h == true %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),['GetObject: ','EMERGENCY STOP']};
                    return
                end
                %self.calcDobot.model.animate(calcTraj(i,:)); % Remove TODO
                self.robot.model.animate(modelTraj(i,:));
                drawnow()
            end
        end
        
        %% MoveObject - Moves the object with the robot end effector to a set location
        function MoveObject(self, object, q, steps, endEffector)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveObject: ','Called']};
        
            q = self.calcDobot.model.ikcon(object.goalLocation,q); 
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveObject: Set of joints to pick up object at ',self.L.MatrixToString(object.goalLocation),' = ',self.L.MatrixToString(q)]};
        
            tr = self.calcDobot.model.fkine(q);
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveObject: Checking result using fkine: ', self.L.MatrixToString(tr)]};
        
            newQ = CalcDobotTo6Dof(q, endEffector);
            
            modelTraj = jtraj(self.robot.model.getpos,newQ,steps);
            if endEffector > 0
                objectRotationTraj = (1/endEffector):((endEffector-(1/endEffector))/steps):endEffector;
            end
        
            for i = 1:steps
                if self.h == true %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveObject: ','EMERGENCY STOP']};
                    return
                end
                self.robot.model.animate(modelTraj(i,:));
                modelTr = self.robot.model.fkine(modelTraj(i,:));
                if endEffector > 0 
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
            figure(1)    
            GetObject(self, transl(location), q, 50);
            liquid = surf([location(1,1)-0.03,location(1,1)-0.03;location(1,1)-0.03,location(1,1)-0.03],[location(1,2)+0.03,location(1,2)+0.03;location(1,2)+0.05,location(1,2)+0.05],[1,location(1,3)-0.15;1,location(1,3)-0.15],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.9,'EdgeColor',colour);
            pause(3);
            delete(liquid);
            UpdateCup(self, teaStage);
        end
        
        %% UpdateCup - Update the appearance of the cup to match contents
        function UpdateCup(self, teaStage)
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
                    AvoidCollisions(self.robot, self.radii, self.centerPoint, qGoal, itemPoints, self.L, self.h);
                case 3                                                      % Test RMRC
                    StirTea(self);
                    %RMRC(self.robot.model, self.cups{1}, transl(self.WATER_LOCATION), self.L, self.h, self.debug);
                case 4
                    % @Sam, add your visual servoing code here to test and
                    % call a.TestFunction(4)
                    q0 = MoveToCoasterArea(self,self.qz,100);
                    VisualServoing(self,q0,self.coasters{self.orderCount})
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

%% CalcDobotTo6Dof - Used to simplfy the calculations on the Dobot due to the hardware limitations of the actual Dobot
function plotQ = CalcDobotTo6Dof(CalcDobotQ, endEffector)
%     % Apply end effector offset for the actual dobot
%     L(1) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',0, 'qlim',[deg2rad(5),deg2rad(80)]);
%     L(2) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-5),deg2rad(85)]);
%     robot = SerialLink(L,'name','test');
%     robot.base = robot.base * trotx(pi) * trotz(-pi/2);
%     initialQ = [CalcDobotQ(3),CalcDobot(4)];
%     trBefore = robot.fkine(initialQ)
%     % Apply end effector offset
%     trAfter = trBefore;
%     trAfter(1,4) = trAfter(1,4) - 0.06; % End effector reach is 0.06, should probably be a const
%     trAfter(2,4) = trAfter(2,4) + 0.1;  % End effector height is 0.1, should probably be a const
%     % Find new solution for offset end effector, ignore rotation and z
%     newQ = robot.ikine(trAfter,initialQ,[1,1,0,0,0,0])

    plotQ = zeros(1,6);
    plotQ(1) = CalcDobotQ(1);
    plotQ(2) = CalcDobotQ(2);
    plotQ(3) = CalcDobotQ(3); % plotQ(3) = newQ(1)
    plotQ(4) = CalcDobotQ(4); % plotQ(4) = newQ(2)
    plotQ(5) = pi/2 - CalcDobotQ(4) - CalcDobotQ(3);
    plotQ(6) = endEffector;
end

%% Collision Detection - derrived from Lab6Solution
function result = IsCollision(robot, radii, centerPoint, qMatrix, points, L, h)
    L.mlog = {L.DEBUG,mfilename('class'),['IsCollision: ','Called']};

    if h == true %Check for emergency stop
        L.mlog = {L.DEBUG,mfilename('class'),['IsCollision: ','EMERGENCY STOP']};
        return
    end

    result = false;
    for i=1:size(qMatrix,1) %% check 
        newQMatrix(i,:) = CalcDobotTo6Dof(qMatrix(i,:),0);
    end
  
    for qIndex = 1:size(newQMatrix,1)

%         % Get link poses for all links
%         q = qMatrix(qIndex,:); %get pos?
%         tr = zeros(4,4,robot.model.n+1);
%         tr(:,:,1) = robot.model.base;
%         l = robot.model.links;
%         for i = 1:robot.model.n
%             if i == 1
%                 tr(:,:,i+1) = tr(:,:,i) * trotz(l(i).theta) * transl(0,0,q(i)) * transl(l(i).a,0,0) * trotx(l(i).alpha); % prismatic joint
%             else
%                 tr(:,:,i+1) = tr(:,:,i) * trotz(q(i) + l.offset) * transl(0,0,l(i).d) * transl(l(i).a,0,0) * trotx(l(i).alpha);
%             end
%         end

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
function AvoidCollisions(robot, radii, centerPoint, qGoal, points, L, h)
    L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','Called']};

    if h == true %Check for emergency stop
        L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','EMERGENCY STOP']};
        return
    end

    %robot.animate(q1);
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
            if ~IsCollision(robot, radii, centerPoint, qMatrixJoin, points, L, h)
                qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                %robot.model.animate(qMatrixJoin); %% Remove??
                size(qMatrix)
                isCollision = false;
                checkedTillWaypoint = i+1;
                % Now try and join to the final goal (qGoal)
                qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); qGoal],deg2rad(10));
                if ~IsCollision(robot, radii, centerPoint, qMatrixJoin, points, L, h)
                    qMatrix = [qMatrix;qMatrixJoin];
                    % Reached goal without collision, so break out
                    break;
                end
            else
                % Randomly pick a pose that is not in collision
                qRand = (2 * rand(1,4) - 1) * pi;
                while (IsCollision(robot, radii, centerPoint, qRand, points, L, h) || ~WithinLimits(robot, CalcDobotTo6Dof(qRand,0))) %%which robot to use for limits????
                    qRand = (2 * rand(1,4) - 1) * pi;
                end
                qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                isCollision = true;
                break;
            end
        end
    end
    for i=1:size(qMatrix,1)
        newQMatrix = CalcDobotTo6Dof(qMatrix(i,:),0);
        robot.model.animate(newQMatrix);
        pause(0.05);
    end

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
%% Moves robot to coaster area before starting visual servoing
function IdealQs = MoveToCoasterArea(self, q, steps)
    self.L.mlog = {self.L.DEBUG,mfilename('class'),['MoveToCoasterArea: ','Called']};

    %put robot in an area in the coaster area to start visual servoing
    IdealQs = [-0.612,-2.3,pi/4,pi/4];
    %IdealQs = [-0.6023 -2.3562 1.3963 0.2437]; %to find target points on image

    newQ = CalcDobotTo6Dof(IdealQs,0);

    %calcTraj = jtraj(self.calcDobot.model.getpos,q,steps); % Remove TODO
    modelTraj = jtraj(self.robot.model.getpos,newQ,steps);

    for i = 1:steps
        if self.h == true %Check for emergency stop
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['GetObject: ','EMERGENCY STOP']};
            return
        end
        %self.calcDobot.model.animate(calcTraj(i,:)); % Remove TODO
        self.robot.model.animate(modelTraj(i,:));
        drawnow()
    end
    
end
%% Moves robot with object to ideal starting position to start visual servoing
% function MoveToFindCoaster(model, object, q, steps, L)
%     L.mlog = {L.DEBUG,mfilename('class'),['MoveToFindCoaster: ','Called']};
% 
%     idealStartQs = [0 1.5609 0.7854 0.7854 0 0];
% 
%     modelTraj = jtraj(model.getpos,idealStartQs,steps);
%     for i = 1:steps
%         model.animate(modelTraj(i,:));
%         modelTr = model.fkine(modelTraj(i,:));
%         modelTr(1:3,1:3) = eye(3); %Don't change object's rotation
%         object.Move(modelTr);
%         drawnow()
%     end
% end

%% Visual servoing code to find coaster
function VisualServoing(self,q0,object) %adapted from Lab 8 visual servoing code
            % Create image target (coaster in the centre of the image plane)           
            coasterStar = [658.1592 768.4425 658.1581 553.5236; 
                          742.7269 642.3605 535.1427 635.6845];
            %coasterStar = [512; 512]; %centre of the image
            halfpointdistance = 0.05;
%             coaster = [-0.67,-0.67,-0.77,-0.77;
%             -3.6,-3.5,-3.5,-3.6;
%             1.04,1.04,1.04,1.04];
            %cam.plot(object.currentLocation(1:3,4))
            coasterpoints = [object.currentLocation(1,4)+halfpointdistance,object.currentLocation(1,4)+halfpointdistance,object.currentLocation(1,4)-halfpointdistance,object.currentLocation(1,4)-halfpointdistance;
                            object.currentLocation(2,4)-halfpointdistance,object.currentLocation(2,4)+halfpointdistance,object.currentLocation(2,4)+halfpointdistance,object.currentLocation(2,4)-halfpointdistance;
                            object.currentLocation(3,4),object.currentLocation(3,4),object.currentLocation(3,4),object.currentLocation(3,4)];
            figure(1)
            plot_sphere(coasterpoints,0.02,'b')
%             matrix=[1 0 0 -0.7000;
%             0 0 -1 -3.5425;
%             0 1 0 1.3147;
%             0 0 0 1.0000];
            %coasterpoints = object.currentLocation(1:3,4)
            %Add the camera (specs similar to Lab 8 visual servoing code)
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', 'DOBOTcam','sensor',0.3);
            % frame rate
            fps = 50;

            %Define values
            %gain of the controler
            lambda = 0.6;
            %depth of the IBVS
            depth = mean (coasterpoints(1,:));

            self.calcDobot.model.plot(q0,'workspace',self.calcDobot.workspace,'nojoints');
            Tc0 = self.calcDobot.model.fkine(q0);
            Tc0 = Tc0*trotx(-pi/2);

            %plot camera
            cam.T = Tc0;
            cam.plot_camera('Tcam',Tc0 ,'label','scale',0.05);
            lighting gouraud
            light
            
            %Display in image view
            %Project points to the image
            p = cam.plot(coasterpoints, 'Tcam', Tc0);
            %camera view and plotting
            cam.clf()
            cam.plot(coasterStar, '*'); % create the camera view
            cam.hold(true);
            cam.plot(coasterpoints, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            cam.hold(true);
            cam.plot(coasterpoints);    % show initial view
            %cam.plot(object.currentLocation(1:3,4))

            %Initialise display arrays
            vel_p = [];
            uv_p = [];
            history = [];

            %% 1.4 Loop
            % loop of the visual servoing
            ksteps = 0;
             while true
                    ksteps = ksteps + 1;
                    
                    % compute the view of the camera
                    uv = cam.plot(coasterpoints);
                    
                    % compute image plane error as a column
                      e = coasterStar-uv;  % feature error
%                     e = uv-coasterStar;  % feature error
                    e = e(:);
                    Zest = [];
                    

                    % compute the Jacobian
                    if isempty(depth)
                        % exact depth from simulation (not possible in practice)
                        pt = homtrans(inv(Tcam), coasterpoints);
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
                    q = q0' + (1/fps)*qp
                    self.calcDobot.model.animate(q');
            
                    %Get camera location
                    Tc = self.calcDobot.model.fkine(q);
                    Tc = Tc*trotx(-pi/2);
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
            figure()            
            plot_p(history,coasterStar,cam)
            figure()
            plot_camera(history)
            figure()
            plot_vel(history)
            figure()
            plot_robjointpos(history)
            figure()
            plot_robjointvel(history)
            
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
    plot(vel(:,1:3), '-')
    hold on
    plot(vel(:,4:6), '--')
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
    plot(vel(:,1:6), '-')
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
    plot(pos(:,1:6), '-')
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