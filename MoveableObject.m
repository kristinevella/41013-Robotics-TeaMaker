classdef MoveableObject < handle
    %MOVEABLEOBJECTS A way of creating a group of moveable objects
    %   The objects are created from .ply files
       
    properties     
        %> Mesh of the object
        mesh;

        %> Vertices of the object
        vertices;
        tVertices;

        %> Initial location of the object
        currentLocation;

        %> Goal location of the object
        goalLocation;
    end
    
    methods
        %% ...structors
        function self = MoveableObject(fileName)
            self.mesh = PlaceObject(fileName);
            self.vertices = get(self.mesh,'Vertices');
        end   
        
        %% Move
        % Move the object
        function Move(self,location)
            transformedVertices = [self.vertices,ones(size(self.vertices,1),1)] * location';
            set(self.mesh,'Vertices',transformedVertices(:,1:3));
            self.tVertices = transformedVertices(:,1:3);
            self.SetCurrentLocation(location);
        end    

        %% SetCurrentLocation
        function SetCurrentLocation(self,location)
            self.currentLocation = location;
        end 
        
        %% SetGoalLocation
        function SetGoalLocation(self,location)
            self.goalLocation = location;
        end 

    end
end

