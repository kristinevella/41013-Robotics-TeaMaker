classdef Assignment2Starter < handle
    properties (Constant)
        BENCH_HEIGHT = 1; % Height of kitchen bench
    end
    properties
        %Logger
        L = SingleInstance.Logger;
        
        % Dobot Magician
        robot;

        % Interactive objects
        kettle;
        cup;
        teaBag;
        sugar;
        spoon;
        milk;

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
            %surf([-3,-3;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap'); %Floor
            %surf([-3,-3;-3,-3],[-1.8,1.8;-1.8,1.8],[0,0;2.7,2.7],'CData',flip(imread('lab1.jpg')),'FaceColor','texturemap'); %Back Wall
            %surf([-3,1.8;-3,1.8],[-1.8,-1.8;-1.8,-1.8],[0,0;2.7,2.7],'CData',flip(imread('lab2.jpg')),'FaceColor','texturemap'); %Side Wall
            

            %Kitchen
            PlaceObject('KitchenBench.ply', [-2.5,0,0]); % Dimensions L(y):7 W(x):1 H(z):1
            PlaceObject('UpperCabinet1.ply', [-2.75,2.7,1.75]); % Dimensions L(y):1 W(x):0.75 H(z):0.75
            PlaceObject('UpperCabinet2.ply', [-2.75,1.7,1.75]); % Dimensions L(y):1 W(x):0.75 H(z):0.75
            PlaceObject('Fridge.ply', [-2.5,3.22,0]); % Dimensions L(y):1.25 W(x):1 H(z):2.25
            
            % Safety Features
            %PlaceObject('ESBtable.ply', [-1.1,-1.2,1]);
            %PlaceObject('FE.ply', [-2.8,1.5,0.39]);

            %PlaceObject('SeatedGirl.ply', [-2.5,-1.2,0])

            self.kettle = MoveableObject('kettle.ply');
            self.kettle.Move(transl(-3,2,1));

            self.cup = MoveableObject('cup.ply');
            self.cup.Move(transl(-3,2,2)); %Starts inside Upper Cabinet 2 (left cabinet)
          
            self.spoon = MoveableObject('spoon.ply');
            self.spoon.Move(transl(-3,2.8,1.05)); %Starts on the bench for now

            self.teaBag = MoveableObject('teabag.ply');
            self.teaBag.Move(transl(-3,2.6,1.15)); %Starts on the bench for now

            self.sugar = MoveableObject('sugarcontainer.ply'); %Added a sugar container, should this become an unmovable object and then add movable sugar cubes?
            self.sugar.Move(transl(-3.2,2.2,1));

            self.milk = MoveableObject('milk.ply'); %Starts inside the fridge
            self.milk.Move(transl(-3,4,1.15));

            % Initialise robot
            self.robot = LinearDobot(false);
          
            axis equal
            camlight
        end
        
        % Make tea using the robot
        function MakeTea(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
    
            % 1. Boil kettle

            % 2. Place cup (open cabinet, get cup, close cabinet)

            % EXAMPLE - Won't work until a cup is added 
            %q = zeros(1,6);
            %GetObject(self.robot, self.cup.currentLocation, q, 100, self.L); % Get the cup
            %MoveObject(self.robot, self.cup, q, 100, self.L) % Pick up cup and move to desired location (TODO: Implement moving the orientation)

            % 3. Place teabag (open container, get bag, close container)

            % 4. Place sugar in cup (...)

            % 5. Pour water

            % 6. Replace kettle

            
            disp('Task 1 - Build model and environment: Complete');
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Complete']};
        end
    end
end

% Moves the end effector of the robot model to a set position
function GetObject(model, location, q, steps, L)
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: ','Called']};

    q = model.ikine(location,q)
    L.mlog = {L.DEBUG,mfilename('class'),['GetObject: Set of joints to pick up object at ',L.MatrixToString(location),' = ',L.MatrixToString(q)]};

    tr = model.fkine(q)
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

    q = model.ikine(brick.goalLocation,q)
    L.mlog = {L.DEBUG,mfilename('class'),['MoveObject: Set of joints to move object to ',L.MatrixToString(brick.goalLocation),' = ',L.MatrixToString(q)]};

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
