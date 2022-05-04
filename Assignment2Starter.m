classdef Assignment2Starter < handle
    properties (Constant)
        WATER_LOCATION = [-1,-2.3,1.2];
        SKIM_MILK_LOCATION = [-1,-2.7,1.2];
        REGULAR_MILK_LOCATION = [-1,-3,1.2];
        ENGLISH_BREAKFAST_LOCATION = [-1,-1.7,1.2];
        GREEN_TEA_LOCATION = [-0.7,-1.7,1.2];
        LEMON_GINGER_TEA_LOCATION = [-0.4,-1.7,1.2];
    end
    properties
        %Logger
        L = SingleInstance.Logger;
        
        % Dobot Magician
        robot;

        % Interactive objects
        cup1;
        cup2;
        cup3;
        cup4;
        teaBag;
        sugarcube;
        spoon;
        coaster1;
        coaster2;
        coaster3;
        coaster4;
        sprayBottle;

    end
    methods
        function self = Assignment2Starter() % Constructor
            close all

            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Instantiated']};

            SetUpEnvironment(self);
        end

        function SetUpEnvironment(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['SetUpEnvironment: ','Called']};
            disp('Initialising the environment...')
            
            hold on
            % Surrounding Surfaces
            surf([-2,-2;4,4],[-4,4.5;-4,4.5],[0.01,0.01;0.01,0.01],'CData',imread('tiles.jpg'),'FaceColor','texturemap'); %Floor
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

            %PlaceObject('SeatedGirl.ply', [-2.5,-1.2,0])

            PlaceObject('hotwaterdispenser.ply', self.WATER_LOCATION); % Set origin at the tap

            PlaceObject('milkdispenserV2.ply', self.SKIM_MILK_LOCATION); % Set origin at the tap
            PlaceObject('milkdispenserV2.ply', self.REGULAR_MILK_LOCATION); % Set origin at the tap

            PlaceObject('teaContainer_EnglishBreakfast.ply',self.ENGLISH_BREAKFAST_LOCATION);
            PlaceObject('teaContainer_Green.ply',self.GREEN_TEA_LOCATION);
            PlaceObject('teaContainer_LemonAndGinger.ply',self.LEMON_GINGER_TEA_LOCATION);

            PlaceObject('sugarcontainer.ply',[-1.2,-3.5,1]); %% TODO Move (out of reach)

            self.cup1 = MoveableObject('cup.ply');
            self.cup1.Move(transl(-0.3,-2.5,1.04));

            self.cup2 = MoveableObject('cup.ply');
            self.cup2.Move(transl(-0.3,-3.1,1.04));

            self.cup3 = MoveableObject('cup.ply');
            self.cup3.Move(transl(-0.3,-2.9,1.04));

            self.cup4 = MoveableObject('cup.ply');
            self.cup4.Move(transl(-0.3,-2.7,1.04));

            self.coaster1 = MoveableObject('coaster.ply'); %% Coasters are out of reach
            self.coaster1.Move(transl(-0.20,-3.6,1.04));

            self.coaster2 = MoveableObject('coaster.ply');
            self.coaster2.Move(transl(-0.45,-3.6,1.04));

            self.coaster3 = MoveableObject('coaster.ply');
            self.coaster3.Move(transl(-0.70,-3.6,1.04));

            self.coaster4 = MoveableObject('coaster.ply');
            self.coaster4.Move(transl(-0.95,-3.6,1.04));
          


%             Tea bag will appear once collected from box?
%             self.teaBag = MoveableObject('teabag.ply');
%             self.teaBag.Move(transl(-3,2.6,1.15)); %Starts on the bench for now

            self.sugarcube = MoveableObject('sugarcube.ply'); 
            self.sugarcube.Move(transl(-1.2,-3.5,1.05));

            self.sprayBottle = MoveableObject('sprayBottle.ply'); % spray bottle will be moved around workspace and dobot must avoid collision
            self.sprayBottle.Move(transl(-0.2,-3.3,1));

            % Initialise robot
            self.robot = Dobot(false);
          
            axis equal
            camlight
        end
        
        % Make tea using the robot
        function MakeTea(self, teaType, milkType, sugarQuantity)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
         
            %% 1. Pickup cup and place under the water dispenser
            q = self.robot.model.getpos(); % ** Change q to suit 
            GetObject(self.robot.model, self.cup1.currentLocation, q, 50, self.L); % Get the cup TODO change to allow multiple cups 
            
            self.cup1.goalLocation = transl(self.WATER_LOCATION);
            q = self.robot.model.getpos(); % ** Change q to suit
            MoveObject(self.robot.model, self.cup1, q, 50, self.L); % Pick up cup and move to under water dispenser

            % TODO dispense water? Press button, water flows down to cup?

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
                    % Cancel order ?? TODO
            end
            
            q = self.robot.model.getpos(); % ** Change q to suit 
            GetObject(self.robot.model, selectedTeaLocation, q, 50, self.L); % Go to the teabag location
            
            self.teaBag = MoveableObject('teabag.ply'); % Instantiate tea bag
            self.teaBag.Move(selectedTeaLocation);

            self.teaBag.goalLocation = self.cup1.currentLocation;
            q = self.robot.model.getpos(); % ** Change q to suit
            MoveObject(self.robot.model, self.teaBag, q, 100, self.L); % Pick up teabag and place in cup

            %% 3. Get sugar (quantity per UI) and place in cup
            if sugarQuantity > 0                                        % Done only if sugar was requested 
                for i=1:sugarQuantity
                    q = self.robot.model.getpos(); % ** Change q to suit 
                    GetObject(self.robot.model, self.sugarcube.currentLocation, q, 50, self.L); % Go to the sugar canister
                    
                    self.sugarcube.goalLocation = transl(self.cup1.currentLocation);
                    q = self.robot.model.getpos(); % ** Change q to suit
                    MoveObject(self.robot.model, self.sugarcube, q, 100, self.L); % Pick up sugercube and place in cup
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
                    % Cancel order ?? TODO
            end

            if milkType > 0
                q = self.robot.model.getpos(); % ** Change q to suit 
                GetObject(self.robot.model, self.cup1.currentLocation, q, 50, self.L); % Get the cup
                
                self.cup1.goalLocation = transl(selectedMilkLocation);
                q = self.robot.model.getpos(); % ** Change q to suit
                MoveObject(self.robot.model, self.cup1, q, 100, self.L); % Pick up cup and move to under milk dispenser
            end 

            % TODO dispense milk? Press button, milk flows down to cup?
            % (button location?)

            %% 5. Pickup cup and place on appropriate available coaster
            q = self.robot.model.getpos(); % ** Change q to suit 
            GetObject(self.robot.model, self.cup1.currentLocation, q, 50, self.L); % Get the cup
            
            self.cup1.goalLocation = self.coaster1.currentLocation; % will need to change for multiple cups of tea TODO
            q = self.robot.model.getpos(); % ** Change q to suit
            MoveObject(self.robot.model, self.cup1, q, 50, self.L); % Pick up cup and move to coaster
            
            disp('Task 1 - Build model and environment: Complete');
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Complete']};
        end
    end
end

% Moves the end effector of the robot model to a set position
function GetObject(model, location, q, steps, L)
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: ','Called']};

    q = model.ikine(location,q); % TODO change to ikcon ?
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: Set of joints to pick up object at ',L.MatrixToString(location),' = ',L.MatrixToString(q)]};

    tr = model.fkine(q);
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: Checking result using fkine: ', L.MatrixToString(tr)]};

    modelTraj = jtraj(model.getpos,q,steps);

    for i = 1:steps
        model.animate(modelTraj(i,:));
        drawnow()
    end
end

% Moves the object with the robot end effector to a set location
function MoveObject(model, object, q, steps, L)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: ','Called']};

    q = model.ikine(object.goalLocation,q) % TODO change to ikcon ?
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: Set of joints to move object to ',L.MatrixToString(object.goalLocation),' = ',L.MatrixToString(q)]};

    tr = model.fkine(q)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: Checking result using fkine: ', L.MatrixToString(tr)]};

    modelTraj = jtraj(model.getpos,q,steps);
    for i = 1:steps
        model.animate(modelTraj(i,:));
        modelTr = model.fkine(modelTraj(i,:));
        object.Move(modelTr);
        drawnow()
    end
end
