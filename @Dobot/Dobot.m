classdef Dobot < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for Dobot robot simulation
function self = Dobot(useGripper)
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace

        
% robot = 
self.GetDobotRobot();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetDobotRobot
% Given a name (optional), create and return a Dobot robot model
function GetDobotRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['Dobot',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end
    L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link - copied from the LinearUR5 class
    L(1).qlim = [-1.33 0];
    L(2) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-90),deg2rad(90)]);
    L(3) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-85),deg2rad(0)]);
    L(4) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-10),deg2rad(90)]);
    L(5) = Link('d',0,        'a',0.06,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-180),deg2rad(180)]);
    L(6) = Link('d',0.1,      'a',0,      'alpha',0,      'offset',0, 'qlim',[deg2rad(-90),deg2rad(90)]);
    
    self.model = SerialLink(L,'name',name);
    
    % Original LinearUR5 code has the base rotated which persists in this
    % class (due to the orientation of the prismatic link). Rotate the
    % robot when used in its desired environment
    self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DobotLink',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
            % linkIndex     % Used for debugging in finding which link was
            % crashing out when loading
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
            disp(ME_1);
            continue;
        end
    end
end        
    end
end