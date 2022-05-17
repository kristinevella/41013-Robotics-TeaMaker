classdef PhysDobot < handle
    properties
        model;                              % Robot model
        workspace = [-2 2 -2 2 -0.3 2];       
    end
    
    methods%% Class for Dobot robot simulation
        function self = PhysDobot()
            self.GetDobotRobot();
        end

        %% GetDobotRobot
        % Create and return a 3 Link Dobot robot model
        % WITHOUT linear rail. Seperate from the main 'Dobot' as our simulation
        % uses a linear rail which we will not be using in the physical task
        function GetDobotRobot(self)
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['CalcDobot',datestr(now,'yyyymmddTHHMMSSFFF')];
            L(1) = Link('d',0.1392,   'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            L(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            L(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset', 0, 'qlim',[deg2rad(-5),deg2rad(85)]);
            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base;      % Used to modify the robot base, currently unutilised but will keep here for possible future use
        end
    end
end