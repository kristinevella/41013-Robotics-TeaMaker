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
        CUP_TOTAL = 4;
    end
    properties
        %Logger
        L = SingleInstance.Logger;
        
        %Emergency Stop handle
        h;

        % Dobot Magician
        robot;

        % Interactive objects
        cups; % Array of cups
        coasters; % Array of coasters
        teaBag;
        sugarcube;
        spoon;
        sprayBottle;
        sideBarrier;
        frontBarrier;

        orderCount;

    end
    methods
        function self = Assignment2Starter() % Constructor
            close all

            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Instantiated']};
            
            self.orderCount = 1;
            SetUpEnvironment(self);
        end

        % DEBUG
        function TestFunction(self)
            clf
            hold on
            axis equal
            
            self.InitialiseRobot();

            %RMRC(self.robot.model, self.cups{1}, transl(self.WATER_LOCATION), self.L);
            AvoidCollisions(self.robot, self.L, self.h);
        end
        % DEBUG END

        function InitialiseRobot(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['InitialiseRobot: ','Called']};
            self.robot = Dobot(false);
            self.robot.model.base = self.robot.model.base * transl(-0.7,-3.3,1.08) * trotx(pi/2); % Moved implementation of robot location from robot class, to Assignment2Starter
            q = self.robot.model.getpos();
            self.robot.model.animate(q);
        end

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
            %PlaceObject('SeatedGirl.ply', [-2.5,-1.2,0])

            PlaceObject('hotwaterdispenser.ply', self.WATER_LOCATION); % Set origin at the tap

            PlaceObject('milkdispenserV2.ply', self.SKIM_MILK_LOCATION); % Set origin at the tap
            PlaceObject('milkdispenserV2.ply', self.REGULAR_MILK_LOCATION); % Set origin at the tap

            PlaceObject('teaContainer_EnglishBreakfast.ply',self.ENGLISH_BREAKFAST_LOCATION);
            PlaceObject('teaContainer_Green.ply',self.GREEN_TEA_LOCATION);
            PlaceObject('teaContainer_LemonAndGinger.ply',self.LEMON_GINGER_TEA_LOCATION);

            PlaceObject('sugarcontainer.ply',[-1.2,-3.5,1]); %% TODO Move (out of reach)
        
            for i = 1:self.CUP_TOTAL
                self.cups{i} = MoveableObject('cup.ply');
                self.coasters{i} = MoveableObject('coaster.ply');
            end

            self.cups{1}.Move(transl(-0.3,-2.5,1.12));
            self.cups{2}.Move(transl(-0.3,-2.7,1.12));
            self.cups{3}.Move(transl(-0.3,-2.9,1.12));
            self.cups{4}.Move(transl(-0.3,-3.1,1.12));

            self.coasters{1}.Move(transl(-0.20,-3.6,1.04));
            self.coasters{2}.Move(transl(-0.45,-3.6,1.04));
            self.coasters{3}.Move(transl(-0.70,-3.6,1.04));
            self.coasters{4}.Move(transl(-0.95,-3.6,1.04));
       
%             Tea bag will appear once collected from box?
%             self.teaBag = MoveableObject('teabag.ply');
%             self.teaBag.Move(transl(-3,2.6,1.15)); %Starts on the bench for now

            self.sugarcube = MoveableObject('sugarcube.ply'); 
            self.sugarcube.Move(transl(-1.2,-3.5,1.05));

            InitialiseRobot(self);
          
            axis equal
            camlight

            % Set the view in a separate function?
            %view(40,30)
            %camzoom(1.5)
        end

        function PlaceColidableItem(self, location)
            if self.sprayBottle == null
                self.sprayBottle = MoveableObject('sprayBottle.ply'); % Initialise spray bottle
            end
            self.sprayBottle.Move(transl(location)); % [-0.2,-3.3,1]
        end

        function LowerBarriers(self) %% should emergency stop stop this ?
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

        function RaiseBarriers(self) %% should emergency stop stop this ?
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
            if self.frontBarrier.ZData(1,2) && self.sideBarrier.ZData(1,2) == self.BARRIER_HEIGHT_MIN
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
        
        % Make tea using the robot
        function MakeTea(self, teaType, milkType, sugarQuantity)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};

            if self.orderCount > 4
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'No empty cups remaining. Cannot make more tea.']};
                disp('No empty cups remaining. Cannot make more tea, please reset.');
                return;
            end
         
            %% Raise Barriers 
            self.RaiseBarriers();

            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end

            %% 1. Pickup cup and place under the water dispenser
            q = self.robot.model.getpos(); % ** Change q to suit 
            GetObject(self.robot.model, self.cups{self.orderCount}.currentLocation, q, 50, self.L, self.h); % Get a cup
            
            self.cups{self.orderCount}.goalLocation = transl(self.WATER_LOCATION);
            RMRC(self.robot.model, self.cups{self.orderCount}, self.L, self.h);
            %q = self.robot.model.getpos(); % ** Change q to suit
            %MoveObject(self.robot.model, self.cups{self.orderCount}, q, 50, self.L); % Pick up cup and move to under water dispenser

            % TODO dispense water? Press button, water flows down to cup?
            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end

            %% 2. Get appropriate teabag (selected from UI) and place in cup
            switch teaType
                case 1
                    selectedTeaLocation = transl(self.ENGLISH_BREAKFAST_LOCATION);
                case 2
                    selectedTeaLocation = transl(self.GREEN_TEA_LOCATION);
                case 3
                    selectedTeaLocation = transl(self.LEMON_GINGER_TEA_LOCATION);
                otherwise
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Invalid tea request - Order cancelled']};
                    disp('Invalid tea request. Order has been cancelled, please try again');
                    return
            end
            
            q = self.robot.model.getpos(); % ** Change q to suit 
            GetObject(self.robot.model, selectedTeaLocation, q, 50, self.L, self.h); % Go to the teabag location
            
            self.teaBag = MoveableObject('teabag.ply'); % Instantiate tea bag
            self.teaBag.Move(selectedTeaLocation);

            self.teaBag.goalLocation = self.cups{self.orderCount}.currentLocation;
            q = self.robot.model.getpos(); % ** Change q to suit
            MoveObject(self.robot.model, self.teaBag, q, 100, self.L, self.h); % Pick up teabag and place in cup

            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end

            %% 3. Get sugar (quantity per UI) and place in cup
            if sugarQuantity > 0                                        % Done only if sugar was requested 
                for i=1:sugarQuantity
                    q = self.robot.model.getpos(); % ** Change q to suit 
                    GetObject(self.robot.model, self.sugarcube.currentLocation, q, 50, self.L, self.h); % Go to the sugar canister
                    
                    self.sugarcube.goalLocation = transl(self.cups{self.orderCount}.currentLocation);
                    q = self.robot.model.getpos(); % ** Change q to suit
                    MoveObject(self.robot.model, self.sugarcube, q, 100, self.L, self.h); % Pick up sugercube and place in cup

                    if self.h == true %Check for emergency stop
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                        return
                    end
                end
            end 

            %% 4. Pickup cup and place under the appropriate milk dispenser
            % (selected from UI)
            switch milkType
                case 0                                                  % None
                    % Do nothing 
                case 1                                                  % Regular
                    selectedMilkLocation = self.REGULAR_MILK_LOCATION;
                case 2                                                  % Skim
                    selectedMilkLocation = self.SKIM_MILK_LOCATION;
                otherwise
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Invalid milk request - Order cancelled']};
                    disp('Invalid milk request. Order has been cancelled, please try again');
                    return
            end

            if milkType > 0
                q = self.robot.model.getpos(); % ** Change q to suit 
                GetObject(self.robot.model, self.cups{self.orderCount}.currentLocation, q, 50, self.L, self.h); % Get the cup
                
                self.cups{self.orderCount}.goalLocation = transl(selectedMilkLocation);
                RMRC(self.robot.model, self.cups{self.orderCount}, self.L, self.h);
                %q = self.robot.model.getpos(); % ** Change q to suit
                %MoveObject(self.robot.model, self.cups{self.orderCount}, q, 100, self.L); % Pick up cup and move to under milk dispenser
            end 

            % TODO dispense milk? Press button, milk flows down to cup?

            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end

            %% 5. Pickup cup and place on appropriate available coaster (Visual servoing part)
            q = self.robot.model.getpos(); % ** Change q to suit 
            %MoveToFindCoaster(self.robot.model, self.cups{self.orderCount}, q, 100, self.L); % TODO Add emergency stop check 
            MoveToFindCoaster(self.robot.model, self.cups{self.orderCount},q, 100, self.L); % TODO Add emergency stop check 
            q0 = self.robot.model.getpos(); % ** Change q to suit 

            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
            
            %Create coaster in workspace %TODO move to setup part of code
            %coaster = [-0.67;-3.6;1.08]; %TODO change later, using current position to make sure dobot can actually reach
            coaster = [-0.67,-0.67,-0.77,-0.77;
                    -3.6,-3.5,-3.5,-3.6;
                     1.04,1.04,1.04,1.04];
            figure(1)
            plot_sphere(coaster,0.05,'b') 
            %plot_circle(coaster,0.15,'b') %TODO fill colour
            
            VisualServoing(self.robot.model,q0,coaster); %TODO FIX!!!!

            pause
            
            self.cups{self.orderCount}.goalLocation = self.coasters{self.orderCount}.currentLocation;
            RMRC(self.robot.model, self.cups{self.orderCount}, self.L, self.h);
            %q = self.robot.model.getpos(); % ** Change q to suit
            %MoveObject(self.robot.model, self.cups{self.orderCount}, q, 50, self.L); % Pick up cup and move to coaster

            if self.h == true %Check for emergency stop
                self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'EMERGENCY STOP']};
                return
            end
            %% Lower Barriers
            self.LowerBarriers();

            self.orderCount = self.orderCount + 1;

            disp('Task 1 - Build model and environment: Complete');
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Complete']};
        end
    end
end

% Moves the end effector of the robot model to a set position
function GetObject(model, location, q, steps, L, h)
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: ','Called']};

    q = model.ikcon(location,q); 
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: Set of joints to pick up object at ',L.MatrixToString(location),' = ',L.MatrixToString(q)]};

    tr = model.fkine(q);
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: Checking result using fkine: ', L.MatrixToString(tr)]};

    modelTraj = jtraj(model.getpos,q,steps);

    for i = 1:steps
        if h == true %Check for emergency stop
            L.mlog = {L.DEBUG,mfilename('class'),['GetObject: ','EMERGENCY STOP']};
            return
        end
        model.animate(modelTraj(i,:));
        drawnow()
    end
end

% Moves the object with the robot end effector to a set location
function MoveObject(model, object, q, steps, L, h)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: ','Called']};

    q = model.ikcon(object.goalLocation,q)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: Set of joints to move object to ',L.MatrixToString(object.goalLocation),' = ',L.MatrixToString(q)]};

    tr = model.fkine(q)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: Checking result using fkine: ', L.MatrixToString(tr)]};

    modelTraj = jtraj(model.getpos,q,steps);
    for i = 1:steps
        if h == true %Check for emergency stop
            L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: ','EMERGENCY STOP']};
            return
        end
        model.animate(modelTraj(i,:));
        modelTr = model.fkine(modelTraj(i,:));
        object.Move(modelTr);
        drawnow()
    end
end

% Collision Avoidance - derrived from Lab6Solution
function AvoidCollisions(robot, L, h)
    L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','Called']};

    if h == true %Check for emergency stop
        L.mlog = {L.DEBUG,mfilename('class'),['AvoidCollisions: ','EMERGENCY STOP']};
        return
    end

    alpha(0.1); %Remove TODO

    % Create cube - For initial testing only - TODO remove
    % One side of the cube
    [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
    sizeMat = size(Y);
    X = repmat(0.75,sizeMat(1),sizeMat(2));
    oneSideOfCube_h = surf(X,Y,Z);
    
    % Combine one surface as a point cloud
    cubePoints = [X(:),Y(:),Z(:)];
    
    % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
    cubePoints = [ cubePoints ...
                 ; cubePoints * rotz(pi/2)...
                 ; cubePoints * rotz(pi) ...
                 ; cubePoints * rotz(3*pi/2) ...
                 ; cubePoints * roty(pi/2) ...
                 ; cubePoints * roty(-pi/2)];         
             
    % Plot the cube's point cloud         
    cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
    cubePoints = cubePoints + repmat([-2,-3,1],size(cubePoints,1),1);
    cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');

    % New values for the ellipsoid (need to check for different configurations) TODO
    centerPoint = [0,0,0];
    radii{1} = [0.1,0.1,0.1];
    radii{2} = [0.1,0.15,0.1];
    radii{3} = [0.1,0.15,0.1];
    radii{4} = [0.1,0.05,0.07];
    radii{5} = [0.1,0.08,0.05];
    radii{6} = [0.05,0.09,0.1];
    radii{7} = [0.03,0.03,0.03];  

    % DEBUG - for visualising only
    for i = 1:robot.model.n+1                                               % robot links + base
        [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii{i}(1), radii{i}(2), radii{i}(3));
        robot.model.points{i} = [X(:),Y(:),Z(:)];
        warning off
        robot.model.faces{i} = delaunay(robot.model.points{i});    
        warning on;

        robot.model.plot3d([0,0,pi/4,pi/4,0,0],'noarrow','workspace',robot.workspace); %for DEBUG
    end

    robot.model.plot3d([0,0,pi/4,pi/4,0,0],'noarrow','workspace',robot.workspace); %TODO Change workspace??
    % DEBUG end
    
    % All links
    q = robot.model.getpos();
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    for i = 1 : robot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha); %%% What is this doing TODO
    end
    
    % Go through each ellipsoid
    for i = 1: size(tr,3)
        cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
        updatedCubePoints = cubePointsAndOnes(:,1:3);
        algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii{i});
        pointsInside = find(algebraicDist < 1);
        disp(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
    end
end 

% TODO: Need to change initial parameters in RMRC each time depending on
% each unique trajectory to get the smoothest motion that doesn't
% exceed limits or hit singularities
% Resolved Motion Rate Control - Adapted from Lab9Solution_Question1
function RMRC(model, object, L, h)
    L.mlog = {L.DEBUG,mfilename('class'),['RMRC: ','Called']};
    
    if h == true %Check for emergency stop
        L.mlog = {L.DEBUG,mfilename('class'),['RMRC: ','EMERGENCY STOP']};
        return
    end
    
    % Set parameters for the simulation
    t = 5;             % Total time (s)
    deltaT = 0.02;      % Control frequency
    steps = t/deltaT;   % No. of steps for simulation
    delta = 2*pi/steps; % Small angle change
    epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector (More emphasis on linear than angular here)
    
    % Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,6);       % Array for joint anglesR
    qdot = zeros(steps,6);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    positionError = zeros(3,steps); % For plotting trajectory error
    angleError = zeros(3,steps);    % For plotting trajectory error
    
    q = model.getpos();
    T = model.fkine(q);
    
    % Set up trajectory, initial pose
    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    for i=1:steps
        x(1,i) = (1-s(i))*T(1,4) + s(i)*object.goalLocation(1,4); % Points in x
        x(2,i) = (1-s(i))*T(2,4) + s(i)*object.goalLocation(2,4); % Points in y
        x(3,i) = (1-s(i))*T(3,4) + s(i)*object.goalLocation(3,4); % Points in z
        % Maintain downwards facing
        theta(1,i) = pi;                 % Roll angle 
        theta(2,i) = 0;                  % Pitch angle
        theta(3,i) = pi;                 % Yaw angle
    end
     
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,6);                                                            % Initial guess for joint angles
    qMatrix(1,:) = model.ikcon(T,q0);                                           % Solve joint angles to achieve first waypoint
    
    % Track the trajectory with RMRC
    for i = 1:steps-1
        if h == true %Check for emergency stop
            L.mlog = {L.DEBUG,mfilename('class'),['RMRC: ','EMERGENCY STOP']};
            return
        end
        T = model.fkine(qMatrix(i,:));                                          % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error (requires small value for deltaT to work)
        S = Rdot*Ra';                                                           % Skew symmetric! (TODO - Check week 6??????????)
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = model.jacob0(qMatrix(i,:));                                         % Get Jacobian at current joint state (Jacobian with respect to the base, alternatively to the end effector)
        m(i) = sqrt(det(J*J'));
        lambdaMax = 5E-2;
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*lambdaMax;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
        for j = 1:6                                                             % Loop through joints 1 to 6
            if h == true %Check for emergency stop
                L.mlog = {L.DEBUG,mfilename('class'),['RMRC: ','EMERGENCY STOP']};
                return
            end
            if qMatrix(i,j) + deltaT*qdot(i,j) < model.qlim(j,1)                % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
                L.mlog = {L.DEBUG,mfilename('class'),['RMRC: Next joint angle is lower than joint limit: ',num2str(qMatrix(i,j) + deltaT*qdot(i,j)),' < ',num2str(model.qlim(j,1)),' - Motor stopped!']};
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
                L.mlog = {L.DEBUG,mfilename('class'),['RMRC: Next joint angle is greater than joint limit: ',num2str(qMatrix(i,j) + deltaT*qdot(i,j)),' > ',num2str(model.qlim(j,2)),' - Motor stopped!']};
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
        positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
        angleError(:,i) = deltaTheta;                                           % For plotting
    end
    
    % DEBUG
    plot3(x(1,:),x(2,:),x(3,:),'r.','LineWidth',1);
    % DEBUG END
    
    %% Do I need this for loop or can I animate in the other one? Will it make the processing time less obvious?
    for i = 1:steps
        model.animate(qMatrix(i,:));
        modelTr = model.fkine(qMatrix(i,:));
        object.Move(modelTr);
        drawnow()
    end
    
    % DEBUG
    for i = 1:6
        figure(2)
        subplot(3,2,i)
        plot(qMatrix(:,i),'k','LineWidth',1)
        title(['Joint ', num2str(i)])
        ylabel('Angle (rad)')
        refline(0,model.qlim(i,1));
        refline(0,model.qlim(i,2));
        
        figure(3)
        subplot(3,2,i)
        plot(qdot(:,i),'k','LineWidth',1)
        title(['Joint ',num2str(i)]);
        ylabel('Velocity (rad/s)')
        refline(0,0)
    end
    
    figure(4)
    subplot(2,1,1)
    plot(positionError'*1000,'LineWidth',1)
    refline(0,0)
    xlabel('Step')
    ylabel('Position Error (mm)')
    legend('X-Axis','Y-Axis','Z-Axis')
    
    subplot(2,1,2)
    plot(angleError','LineWidth',1)
    refline(0,0)
    xlabel('Step')
    ylabel('Angle Error (rad)')
    legend('Roll','Pitch','Yaw')
    figure(5)
    plot(m,'k','LineWidth',1)
    refline(0,epsilon)
    title('Manipulability')
    % DEBUG END

end 

%Moves robot with object to ideal starting position to start visual servoing
function MoveToFindCoaster(model, object, q, steps, L)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveToFindCoaster: ','Called']};

    idealStartQs = [0 1.5609 0.7854 0.7854 0 0];

    modelTraj = jtraj(model.getpos,idealStartQs,steps);
    for i = 1:steps
        model.animate(modelTraj(i,:));
        modelTr = model.fkine(modelTraj(i,:));
        object.Move(modelTr);
        drawnow()
    end
end

%Visual servoing code to find coaster
function VisualServoing(model,q0,coaster) %adapted from Lab 8 visual servoing code
            % Create image target (coaster in the centre of the image plane)           
            coasterStar = [800 200 200 800; 300 300 800 800];
            %coasterStar = [512; 512]; %centre of the image


            %Add the camera (specs similar to Lab 8 visual servoing code)
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', 'DOBOTcam');
            % frame rate
            fps = 1000;

            %Define values
            %gain of the controler
            lambda = 0.6;
            %depth of the IBVS
            depth = mean (coaster(1,:));

            Tc0 = model.fkine(q0);

            %plot camera
            cam.T = Tc0;
            cam.plot_camera('Tcam',Tc0 ,'label','scale',0.05);
            lighting gouraud
            light
            
            %Display in image view
            %Project points to the image
            p = cam.plot(coaster, 'Tcam', Tc0);
            %camera view and plotting
            cam.clf()
            cam.plot(coasterStar, '*'); % create the camera view
            cam.hold(true);
            cam.plot(coaster, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            cam.hold(true);
            cam.plot(coaster);    % show initial view

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
                    uv = cam.plot(coaster);
                    
                    % compute image plane error as a column
                    e = coasterStar-uv;  % feature error
                    e = e(:);
                    Zest = [];
                    

                    % compute the Jacobian
                    if isempty(depth)
                        % exact depth from simulation (not possible in practice)
                        pt = homtrans(inv(Tcam), coaster);
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
                    J2 = model.jacobn(q0);
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
                    model.animate(q');
            
                    %Get camera location
                    Tc = model.fkine(q);
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
                    pause
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

%% Functions for plotting (From Lab 8 Solution)

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