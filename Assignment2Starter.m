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
        %coaster;
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
                %Glass barrier
            surf([-0.1,-0.1;-0.1,-0.1],[-3.7,-3.7;-1.3,-1.3],[1,2.5;1,2.5],'CData',flip(imread('glass.jpg')),'FaceColor','texturemap','FaceAlpha',0.3,'EdgeColor','none');
            
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

%             self.coaster1 = MoveableObject('coaster.ply'); %% Coasters are out of reach
%             self.coaster1.Move(transl(-0.20,-3.6,1.04));
% 
%             self.coaster2 = MoveableObject('coaster.ply');
%             self.coaster2.Move(transl(-0.45,-3.6,1.04));
% 
%             self.coaster3 = MoveableObject('coaster.ply');
%             self.coaster3.Move(transl(-0.70,-3.6,1.04));
% 
%             self.coaster4 = MoveableObject('coaster.ply');
%             self.coaster4.Move(transl(-0.95,-3.6,1.04));
          


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

            %% 5. Pickup cup and place on appropriate available coaster (Visual servoing part)
            q = self.robot.model.getpos(); % ** Change q to suit 
            GetObject(self.robot.model, self.cup1.currentLocation, q, 50, self.L); % Get the cup

            q = self.robot.model.getpos(); % ** Change q to suit 
            %TODO Move robot to left side of linear rail facing coaster
            %section while holding a cup find q values needed
            MoveToFindCoaster(self.robot.model, self.cup1, q, 100, self.L);
            q = self.robot.model.getpos(); % ** Change q to suit 

            % Create image target (coaster in the centre of the image plane)
            coasterStar = [512; 512];
            %Create coaster as a 3D point
            coaster = [-0.65;-3.5;1.04];
            plot_circle(coaster,0.05,'b') %TODO fill colour

            %Add the camera (specs sismilar to Lab 8)
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', 'DOBOTcam');
            % frame rate
            fps = 25;
            %Define values
            %gain of the controler
            lambda = 0.6;
            %depth of the IBVS
            depth = mean (coaster(1,:));

            Tc0 = self.robot.model.fkine(q);

            %plot camera and points
            cam.T = Tc0;

            %Display camera
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

            pause
            %% 1.4 Loop
            % loop of the visual servoing
            ksteps = 0;
             while true
                    ksteps = ksteps + 1;
                    
                    % compute the view of the camera
                    uv = cam.plot(coaster);
                    
                    % compute image plane error as a column
                    e = coasterStar-uv;   % feature error
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
                    J2 = self.robot.model.jacobn(q);
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
                    q = q + (1/fps)*qp;
                    self.robot.model.animate(q');
            
                    %Get camera location
                    Tc = self.robot.model.fkine(q);
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
            
                    if ~isempty(200) && (ksteps > 200)
                        break;
                    end
                    
                    %update current joint position
                    q = q;
             end %loop finishes
             
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
            pause

            %TODO continue with visual servoing
            
            
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

    q = model.ikcon(location,q); 
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

    q = model.ikcon(object.goalLocation,q)
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