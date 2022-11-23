% If you want to fold the code for each section in YOUR MATLAB go to
%    Editor/Debugger (MATLAB Preferences) > Code Folding and enable "Sections" by checking on it
function PI = Mass_information(robot)
%% Version of the  21 November 2022
% Information for dynamic parameters of the robot

%% I.   Masses - Center of Masses - Inertia Tensors
%  ============================================
nFrames = robot.nFrames;
d1 = robot.geometric_params.d1; % horizontal distance from the foot sole (below the ankle) to its tip
d2 = robot.geometric_params.d2; % vertical distance from the foot sole (below the ankle) to its ankle
l1 = robot.geometric_params.l1; % Lenght of the tibia
l2 = robot.geometric_params.l2; % Lenght of the femur
d3 = robot.geometric_params.d3; %length of torso
d4 = robot.geometric_params.d4; % horizontal length of the hip
% 
% The centers of masses positions and the inertial tensors are described
% relatively to the local coordinate system of the current solid (S) (o, R).
% The inertia tensors are expressed around the mass center. 
% i.e. ^SI0_i where S is a local fixed frame of the body and I0 is the
% inertia tensor
% 
% All solids (S) are described 
% relatively to the zero posture: standing with straight
% 
% Masses are expressed in kg

M = zeros(nFrames,1);

% CoM = [X_G ; Y_G ; Z_G]; kg/m²
%
% I= [I_{xx} , I_{xy} , I_{xz} ;
%     I_{yx} , I_{yy} , I_{yz} ;
%     I_{zx} , I_{zy} , I_{zz}]; kg/m²

%% CoMs and Masses   
% All CoM are refered to a frame located in the same position of the frames for the geometric
% parameters (at joints, upper torso, tip feet), but with the orientation (world frame):
%       y
%       |
%       |
%       |______ x
%      /
%     /
%    /
%   z
%   
% Note: The distances are taken with the the robot is in zero position

% Body: Right Foot    
% Local frame at: RFoot Tip
M(1) = 1;

CoM{1} = [ -d1/2;
            d2/2;
             0];

I{1}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Tibia
% Local frame at: RAnkle
M(2) = 1;

CoM{2} = [ 0;
          l1/2;
         0];

I{2}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Femur
% Local frame at: RKnee 
M(3) = 1;

CoM{3} = [ 0;
          l2/2;
         0];

I{3}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Hip
% Local frame at: RHip
M(4) = 1;

CoM{4} = [ 0;
          0;
         -d4/2];

I{4}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Trunk
% Local frame at: Torso 
M(5) = 1;

CoM{5} = [ 0;
          -d3/2;
         0];

I{5}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Left femur (asigned to left hip)
% Local frame at: Lhip 
M(6) = 1;

CoM{6} = [ 0;
          -l2/2;
         0];

I{6}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------

% Body: Left Tibia 
% Local cordinate: LKnee
M(7) = 1;

CoM{7} = [ 0;
          -l1/2;
         0];

I{7}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: NO BODY (it is easier to define the CoM of the foot from other frame attached to it)
% Local cordinate: LAnkle
M(8) = 0;

CoM{8} = [ 0;
          0;
         0];

I{8}= [0, 0, 0;
       0, 0, 0;
       0, 0, 0];
% ------
% Body: Right Foot    
% Local cordinate: LFoot Tip
M(9) = 1;

CoM{9} = [ -d1/2;
          d2/2;
         0];

I{9}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: NO BODY (asigned to sole of left foot)
% Local frame at: LSoleFoot 
M(10) = 0;

CoM{10} = [ 0;
          0;
         0];

I{10}= [0, 0, 0;
       0, 0, 0;
       0, 0, 0];
% ------

%////////////////////////////////////////////////////////////////////////

%% III. Transformation matrices w.r.t frame 0 (Tip of the RFoot)
robot.q = zeros(robot.joints,1);
robot.T = DGM(robot);
T0 = robot.T;

%% IV.  Calculation of I. in M-DH frames

% Base change I_j = inv(P_j) I_j_s P_j

% Transport of inertia
% I_j = I_j_s + M Y_s
% with Y_s = [b^2+c^2,-a*b,-a*c;
%             -a*b,c^2+a^2,-b*c;
%             -a*c,-b*c,a^2+b^2]:
% Knowing CoM = [a;b;c];

% Transport of the CoM 
% d_xg = d_x - d_x_0
% d_yg = d_y - d_y_0
% d_zg = d_z - d_z_0

% % Rotation matrices 
P = cell(1,nFrames);
for i = 1:nFrames
	P{i} = inv(T0(1:3,1:3,i));  % P = jR0
end

% % Translation of the frames
% vect_C are the translation vectors to make the CoM be refered to the origin of the frame set DH- frames with
% the geometic parameters (Denavit Hartenberg DH)
% In this case, all the origins of CoM vectors coincide with the DH frames, so there's no need to
% traslate them.
vec_C = cell(1,nFrames);
for i = 1:nFrames
    vec_C{i} = [0;0;0];
end

% Now we refered all CoM vectors and Inercia tensors w.r.t. the DH-Frames
for i = 1 : nFrames
    C = vec_C{i}; %vec_C contains the traslation vectors of the reference frames;
    inter = CoM{i};
    tmp=[inter(1)-C(1);inter(2)-C(2);inter(3)-C(3)];
    CoM{i} = P{i}*tmp; % Computation of the position of the CoM, w.r.t. the DH reference frames

    Y = [tmp(2)^2+tmp(3)^2,-tmp(1)*tmp(2),-tmp(1)*tmp(3);
     -tmp(1)*tmp(2),tmp(3)^2+tmp(1)^2,-tmp(2)*tmp(3);
     -tmp(1)*tmp(3),-tmp(2)*tmp(3),tmp(1)^2+tmp(2)^2];
    
    % Tralation of the inertia tensor
    I{i} = I{i} + M(i) * (Y);   % application of translation  % SIj = bIb + Mj*Yj  "S" is the "World" frame 
    % Rotation of the inertia tensor, to be refered to the DH frames
    I{i} = P{i} * I{i} * P{i}'; % application of rotation % jIj = jRb bIj bRj 
end

% % ==============================================================================================
% % The NEXT part of the code is added in order to concentrate the mass of the robot in one point
% % ==============================================================================================
global alphaM % alphaM = [0-1]. Allow to remove gradually the value of all masses and inertias of the robot
% alphaM = 0 -> all masses are 0 -> all mass is concentrated in one point
% alphaM = 1 -> all masses are normal -> all masses are distributed
% If alphaM is empty it means it was not initialized by other codes, therefore it is not required to do this 
if ~isempty(alphaM) 
    if alphaM > 1
        alphaM = 1;
    elseif alphaM < 0
        alphaM = 0;
    end
    
    MassTorso = 0;
    for i = 1:nFrames
        I{i} = alphaM*I{i};
        MassTorso = MassTorso + (1-alphaM)*M(i);
        M(i) = alphaM*M(i);
    end
    
    % % Assignment of the masss to the frame 5 (Torso),
    M(5)= M(5) + MassTorso;% Remaining of the mass asigned to frame 5
    % If less than 20% of the mass is concentrated in the frame 5 the same CoM position (of frame 5) will be used, but...
    if alphaM < 0.75
        CoM{5} = [d3; 0; 0]; % we change the CoM position of frame 15 if more than 20% of the mass are concentred in this frame...
    end
end
% % ==============================================================================================
PI.mass = M;
PI.CoM = CoM; % ^jCoM, it is the CoM of link j w.r.t. frame j
PI.I = I; %  ^jIj i.e. is the inertia tensor of link j, located in the origin of frame j

 