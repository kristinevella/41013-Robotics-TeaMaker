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
        mug;
        teaBag;
        sugar;

    end
    methods
        function self = Assignment2Starter() % Constructor
            close all

            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Instantiated']};

            SetUpEnvironment;
        end

        function SetUpEnvironment(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),['SetUpEnvironment: ','Called']};
            disp('Initialising the environment...')
            
            hold on
            % Surrounding Surfaces
            %surf([-3,-3;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap'); %Floor
            %surf([-3,-3;-3,-3],[-1.8,1.8;-1.8,1.8],[0,0;2.7,2.7],'CData',flip(imread('lab1.jpg')),'FaceColor','texturemap'); %Back Wall
            %surf([-3,1.8;-3,1.8],[-1.8,-1.8;-1.8,-1.8],[0,0;2.7,2.7],'CData',flip(imread('lab2.jpg')),'FaceColor','texturemap'); %Side Wall
            
            %PlaceObject('KitchenBench.ply', [0,0,0.01]); % TODO
            
            % Safety Features
            %PlaceObject('ESBtable.ply', [-1.1,-1.2,1]);
            %PlaceObject('FE.ply', [-2.8,1.5,0.39]);

            %PlaceObject('SeatedGirl.ply', [-2.5,-1.2,0])

            %self.kettle = MoveableObject('Kettle.ply'); % TODO
            %self.kettle.Move(transl(1,1,1));

            %self.mug = MoveableObject('Mug.ply'); % TODO
            %self.mug.Move(transl(1.1,1,1));

            %self.teaBag = MoveableObject('TeaBag.ply'); % TODO
            %self.teaBag.Move(transl(1.2,1,1));

            %self.sugar = MoveableObject('Sugar.ply'); % TODO
            %self.sugar.Move(transl(1.3,1,1));

            % Initialise robot here....
            %self.robot = % Create instance of robot class % TODO
          
            axis equal
            camlight
        end
        
        % Make tea using the robot
        function MakeTea(self)
            self.L.mlog = {self.L.DEBUG,mfilename('class'),[self.L.Me,'Called']};
    
            % 1. Boil kettle

            % 2. Place mug (open cabinet, get mug, close cabinet)

            % EXAMPLE - Won't work until a mug is added 
            %q = zeros(1,6);
            %GetObject(self.robot, self.mug.currentLocation, q, 100, self.L); % Get the mug
            %MoveObject(self.robot, self.mug, q, 100, self.L) % Pick up mug and move to desired location (TODO: Implement moving the orientation)

            % 3. Place teabag (open container, get bag, close container)

            % 4. Place sugar in mug (...)

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
