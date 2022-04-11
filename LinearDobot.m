classdef LinearDobot < handle
    properties
        L = SingleInstance.Logger;

        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for UR5 robot simulation
function self = LinearDobot(useGripper)
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace

        
% robot = 
self.GetDobotRobot();
% robot = 
%self.PlotAndColourRobot();
self.model.plot(zeros(1,self.model.n),'noarrow','workspace',self.workspace); %% Temporary until robot model files are included (@Juan)
end

%% GetDobotRobot
% Given a name (optional), create and return a UR5 robot model
function GetDobotRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['LinearDobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    % Create the Dobot model mounted on a linear rail
    %% These are from the linear UR5 and need to be changed!!!!
    L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
    L(2) = Link([0      0.1599  0       -pi/2   0]);
    L(3) = Link([0      0.1357  0.425   -pi     0]);
    L(4) = Link([0      0.1197  0.39243 pi      0]);
    L(5) = Link([0      0.093   0       -pi/2   0]);
    L(6) = Link([0      0       0       0	    0]);
    
    % Incorporate joint limits (Suggested limits from notes on Canvas)
    L(1).qlim = [-0.8 0]; % PRISMATIC Link
    L(2).qlim = [-135 135]*pi/180;
    L(3).qlim = [5 80]*pi/180;
    L(4).qlim = [15 170]*pi/180;
    L(5).qlim = [-90 90]*pi/180;
    L(6).qlim = [-85 85]*pi/180;

    L(3).offset = -pi/2;
    
    self.model = SerialLink(L,'name',name);
    
    % Rotate robot to the correct orientation and position
    self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
    self.L.mlog = {self.L.DEBUG,mfilename('class'),['GetDobotRobot: ','A Linear Dobot robot model has been created']};
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinDobotLink',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinDobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            %disp(ME_1);
            self.L.mlog = {self.L.WARN,mfilename('class'),['PlotAndColourRobot: ','There was a warning',self.L.ExceptionToString(ME_1)]};
            continue;
        end
    end
    self.L.mlog = {self.L.DEBUG,mfilename('class'),['PlotAndColourRobot: ','The Linear Dobot has been plotted']};
end        
    end
end