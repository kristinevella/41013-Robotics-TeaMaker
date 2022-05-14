% Initialise the robot
robot = ThreeLinkDobot(false);
workspace = [-0.4 0.4 -0.4 0.4 -0.1 0.4];
robot.model.base = robot.model.base * trotx(pi/2);

%%
CalcDobotQ = [0, 0, 45, 45];
CalcDobotQ = deg2rad(CalcDobotQ);

trBefore = robot.model.fkine(CalcDobotQ)

RearArmLength = 0.135;
ForearmLength = 0.147;
endEffectorReach = 0.06;
endEffectorHeight = 0.1;

% calculate the horizontal distance (x) and vertical distance (y) of the
% end effector, to the base of the rear arm
x = RearArmLength * sin(CalcDobotQ(3)) + ForarmLength * sin(CalcDobotQ(3) + CalcDobotQ(4));
y = RearArmLength * cos(CalcDobotQ(3)) + ForarmLength * cos(CalcDobotQ(3) + CalcDobotQ(4));

% Calculate the new required x and y
newX = x - endEffectorReach;
newY = y - endEffectorHeight;

syms q1 q2;
eqn1 = RearArmLength * sin(q1) + ForearmLength * sin(q1 + q2) == newX;
eqn2 = RearArmLength * cos(q1) + ForearmLength * cos(q1 + q2) == newY;

[A,B] = equationsToMatrix([eqn1, eqn2], [q1, q2]);
sol = linsolve(A,B);

%function plotQ = CalcDobotTo6Dof(CalcDobotQ, endEffector)
    plotQ = zeros(1,6);
    plotQ(1) = CalcDobotQ(1);
    plotQ(2) = CalcDobotQ(2);
    plotQ(3) = CalcDobotQ(3);
    plotQ(4) = CalcDobotQ(4);
    plotQ(5) = pi/2 - CalcDobotQ(4) - CalcDobotQ(3);
    plotQ(6) = 0;
%end

trAfter = robot.model.fkine(plotQ(1:4));
