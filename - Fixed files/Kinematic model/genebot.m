function robot = genebot()
%%Generation of the robot datas

%% Number of joints
joints = 6;

%% ZERO joint positions
q = zeros(joints,1); 

%% Joint velocities
qD = zeros(joints,1);

%% Creating the robot structure
robot = struct('joints',joints,'q',q,'qD',qD);

%% Robot antecedent and joint list
robot.ant = [0,... 1
             1,... 2
             2,... 3
             3,... 4
             4,... 5
             5,... 6
             6,... 7
             7,... 8
             8,... 9
             9]; %10  
robot.act= [0,1,2,3,0,4,5,6,0,0];

%% Geometric direct model: Compute all Transformation matrices w.r.t. frame 0 (at stance foot).
robot.q = q;
robot.T = DGM(robot); % At this time this would be "zero position"

%% CONSTANT transformation matrices to convert make all matrices 0Ti be aligned with the world frame (frame 0) at ZERO position
% ------------------------------------------
Tconst = zeros(4,4,9);
for i=1:7
    Tconst(:,:,i) = eye(4);
    R = robot.T(1:3,1:3,i);
    Tconst(1:3,1:3,i) = R';
end    
robot.Tconst = Tconst;

%% Assigning DEFAULT joint positions (move the robot in a stance position)
q(1) = deg2rad(-15);
q(2) = deg2rad(30);
q(3) = deg2rad(-15);
q(4) = deg2rad(15);
q(5) = deg2rad(-30);
q(6) = deg2rad(15);

robot.q = q;
robot.T = DGM(robot); % Modelo geom�trico directo (calcula las matrices de transformaci�n elementales del robot),
                       % y las asigna a la estructura "robot" en la variable T.                       

%% Robot mass information
% PI = Mass_information;
% M = PI.masse;
% robot.mass = sum(M);
% robot.PI = PI;
% 
% [robot.CoM,robot.J_CoM,robot.J_Ankle,robot.crossM,robot.J_CoMs] = compute_com(robot,PI);
% 


end

