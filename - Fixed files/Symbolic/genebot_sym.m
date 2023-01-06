function robot = genebot_sym()
%%Generation of the robot datas

%% Number of joints
joints = 6;

%% joint positions
syms q1 q2 q3 q4 q5 q6
q = [q1; q2; q3; q4; q5; q6];

%% Joint velocities
syms qD1 qD2 qD3 qD4 qD5 qD6
qD = [qD1; qD2; qD3; qD4; qD5; qD6];

%% Creating the robot structure
robot = struct('joints',joints,'q',q,'qD',qD);

%% Robot geometric params

syms d1 d2 dd dAngle l1 l2 d3 d4

robot.nFrames = 10; % 6 for each joint, and + 4, tip foot 1, Torso, tip and sole foot 2
geometric_params.d1 = d1; % horizontal distance from the foot sole (below the ankle) to its tip
geometric_params.d2 = d2;   % vertical distance from the foot sole (below the ankle) to its ankle
geometric_params.dd = dd;
geometric_params.dAngle = dAngle;
geometric_params.l1 = l1; % Lenght of the tibia
geometric_params.l2 = l2; % Lenght of the femur
geometric_params.d3 = d3; %length of torso
geometric_params.d4 = d4; % horizontal length of the hip

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
robot.T = DGM_sym(robot); % At this time this would be "zero position"

%% CONSTANT transformation matrices to convert all matrices 0Ti be aligned with the world frame (frame 0) at ZERO position
% ------------------------------------------
for i=1:robot.nFrames
    R = robot.T(1:3,1:3,i);
    Tconst(1:3,1:3,i) = transpose(R);
    Tconst(4,1:4,i) = [0,0,0,1];
end    
disp('Simplifying Tconst matrices...')
robot.Tconst = simplify(Tconst);
                 
%% Robot mass information
PI = Mass_information_sym(robot);
M = PI.mass;
robot.mass = sum(M);
robot.PI = PI;

% [robot.CoM,robot.J_CoM,robot.J_Ankle,robot.crossM,robot.J_CoMs] = compute_com(robot,PI);




end

