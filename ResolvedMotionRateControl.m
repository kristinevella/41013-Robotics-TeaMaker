classdef ResolvedMotionRateControl
    properties
        L = SingleInstance.Logger;          % Logger
        line_handle;                        % Handle for plotted translation          

        deltaT = 0.02;                      % Control frequency
        steps = 360*2;                      % No. of steps for simulation (2 revolutions)
        epsilon = 0.1;                      % Threshold value for manipulability/Damped Least Squares
        W = diag([1 1 1 0.1 0.1 0.1]);      % Weighting matrix for the velocity vector (More emphasis on linear than angular here)

        m;            % Array for Measure of Manipulability
        qMatrix;       % Array for joint anglesR
        qdot;          % Array for joint velocities
        theta;         % Array for roll-pitch-yaw angles
        x;             % Array for x-y-z trajectory
        positionError; % For plotting trajectory error
        angleError;    % For plotting trajectory error
        T;             % Transformation
    end

    methods
        function self = ResolvedMotionRateControl(calcDobot,model,debug,h)
            self = InitialiseArrays(self);

            q = model.getpos();
            q = q(1,1:4);
            self = SetCircularTrajectory(self, calcDobot.model.fkine(q));         % Create transformation of first point and angle
            q0 = zeros(1,4);                                                % Initial guess for joint angles
            self.qMatrix(1,:) = calcDobot.model.ikcon(self.T,q0);                      % Solve joint angles to achieve first waypoint
            
            % Track the trajectory with RMRC
            for i = 1:self.steps-1
                if h == true %Check for emergency stop
                    self.L.mlog = {self.L.DEBUG,mfilename('class'),['RMRC: ','EMERGENCY STOP']};
                    return
                end
                self.T = calcDobot.model.fkine(self.qMatrix(i,:));                                          % Get forward transformation at current joint state
                deltaX = self.x(:,i+1) - self.T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(self.theta(1,i+1),self.theta(2,i+1),self.theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = self.T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Ra = real(Ra);
                Rdot = (1/self.deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error (requires small value for deltaT to work)
                S = Rdot*Ra';                                                           % Skew symmetric! (TODO - Check week 6??????????)
                linear_velocity = (1/self.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = self.W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = calcDobot.model.jacob0(self.qMatrix(i,:));                                         % Get Jacobian at current joint state (Jacobian with respect to the base, alternatively to the end effector)
                self.m(i) = sqrt(det(J*J'));
                lambdaMax = 5E-2;
                if self.m(i) < self.epsilon  % If manipulability is less than given threshold
                    lambda = (1 - self.m(i)/self.epsilon)*lambdaMax;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(4))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
                for j = 1:4                                                           % Loop through joints 1 to 6
                    if h == true %Check for emergency stop
                        L.mlog = {L.DEBUG,mfilename('class'),['RMRC: ','EMERGENCY STOP']};
                        return
                    end
                    if self.qMatrix(i,j) + self.deltaT*qdot(i,j) < calcDobot.model.qlim(j,1)                % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),['RMRC: Next joint angle is lower than joint limit: ',num2str(self.qMatrix(i,j) + self.deltaT*qdot(i,j)),' < ',num2str(calcDobot.model.qlim(j,1)),' - Motor stopped!']};
                    elseif self.qMatrix(i,j) + self.deltaT*qdot(i,j) > calcDobot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                        self.L.mlog = {self.L.DEBUG,mfilename('class'),['RMRC: Next joint angle is greater than joint limit: ',num2str(self.qMatrix(i,j) + self.deltaT*qdot(i,j)),' > ',num2str(calcDobot.model.qlim(j,2)),' - Motor stopped!']};
                    end
                end
                self.qMatrix(i+1,:) = real(self.qMatrix(i,:) + self.deltaT*qdot(i,:));                       % Update next joint state based on joint velocities
                positionError(:,i) = self.x(:,i+1) - self.T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
            if debug == 1
                for i = 1:4
                    figure(2)
                    subplot(3,2,i)
                    plot(self.qMatrix(:,i),'k','LineWidth',1)
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
                plot(self.m,'k','LineWidth',1)
                refline(0,self.epsilon)
                title('Manipulability')

                figure(1)
            end 
            
        end

        %% InitialiseArrays
        function self = InitialiseArrays(self)
            self.m = zeros(self.steps,1);             % Array for Measure of Manipulability
            self.qMatrix = zeros(self.steps,4);       % Array for joint anglesR
            self.qdot = zeros(self.steps,4);          % Array for joint velocities
            self.theta = zeros(3,self.steps);         % Array for roll-pitch-yaw angles
            self.x = zeros(3,self.steps);             % Array for x-y-z trajectory
            self.positionError = zeros(3,self.steps); % For plotting trajectory error
            self.angleError = zeros(3,self.steps);    % For plotting trajectory error
        end

        %% SetCircularTrajectory
        function self = SetCircularTrajectory(self, T)
            for i=1:self.steps
                self.x(1,i) = T(1,4) + 0.05 + 0.02*cos(self.deltaT*i); % Points in x
                self.x(2,i) = T(2,4) + 0.02 + 0.02*sin(self.deltaT*i); % Points in y
                self.x(3,i) = T(3,4);                           % Points in z
            end
            self.T = [eye(3) self.x(:,1); zeros(1,3) 1]; 
        end

        %% SetStraightTrajectory - NOT USED
        function T = SetStraightTrajectory(self, T, location)
            s = lspb(0,1,self.steps);                                            % Trapezoidal trajectory scalar
            for i=1:self.steps
                self.x(1,i) = (1-s(i))*T(1,4) + s(i)*location(1,4); % Points in x
                self.x(2,i) = (1-s(i))*T(2,4) + s(i)*location(2,4); % Points in y
                self.x(3,i) = (1-s(i))*T(3,4) + s(i)*location(3,4); % Points in z
                % Change to maintain downwards facing
                self.theta(1,i) = 0;              % Roll angle 
                self.theta(2,i) = 0;              % Pitch angle
                self.theta(3,i) = 0;              % Yaw angle
            end
            T = [rpy2r(self.theta(1,1),self.theta(2,1),self.theta(3,1)) self.x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
        end
    end
end