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

%% Robot geometric params

robot.nFrames = 10; % 6 for each joint, and + 4, tip foot 1, Torso, tip and sole foot 2
geometric_params.d1 = 0.2; % horizontal distance from the foot sole (below the ankle) to its tip
geometric_params.d2 = 0.1;   % vertical distance from the foot sole (below the ankle) to its ankle
geometric_params.dd = sqrt(geometric_params.d1^2 + geometric_params.d2^2);
geometric_params.dAngle = atan2(geometric_params.d2,geometric_params.d1);
geometric_params.l1 = 1; % Lenght of the tibia
geometric_params.l2 = 1; % Lenght of the femur
geometric_params.d3 = 1; %length of torso
geometric_params.d4 = 0.3; % horizontal length of the hip

robot.geometric_params = geometric_params;

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
% Direct geometic model of robot. It compute all the elemental matrices w.r.t. frame 0 and asign it
% to robot structure in variable "T".
robot.T = DGM(robot); % At this time this would be "zero position"

%% CONSTANT transformation matrices to convert all matrices 0Ti be aligned with the world frame (frame 0) at ZERO position
% ------------------------------------------
Tconst = zeros(4,4,robot.nFrames);
for i=1:robot.nFrames
    Tconst(:,:,i) = eye(4);
    R = robot.T(1:3,1:3,i);
    Tconst(1:3,1:3,i) = R';
end    
robot.Tconst = Tconst;
                 
% %% Assigning DEFAULT joint positions (move the robot in a stance position)
% q(1) = deg2rad(-15);
% q(2) = deg2rad(30);
% q(3) = deg2rad(-15);
% q(4) = deg2rad(15);
% q(5) = deg2rad(-30);
% q(6) = deg2rad(15);
% 
% robot.q = q;
% robot.T = DGM(robot); % We set some initial stand pose for the robot  


%% Robot mass information
PI = Mass_information(robot);
M = PI.mass;
robot.mass = sum(M);
robot.PI = PI;

[robot.CoM,robot.J_CoM,robot.J_Ankle,robot.crossM,robot.J_CoMs] = compute_com(robot,PI);




end

