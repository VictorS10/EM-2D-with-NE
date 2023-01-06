% If you want to fold the code for each section in YOUR MATLAB go to
%    Editor/Debugger (MATLAB Preferences) > Code Folding and enable "Sections" by checking on it
function PI = Mass_information_sym(robot)
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

syms m_foot m1 m2 m_torso m3 m4 
syms I_foot I1 I2 I_torso I3 I4
syms lc_foot lc1 lc2 lc_torso lc3 lc4 
syms alpha_foot % alpha_foot = atan2(d2,d1);

% Body: Right Foot    
% Local frame at: RFoot Tip
M(1) = m_foot;

CoM{1} = [ -lc_foot*cos(alpha_foot);
            lc_foot*sin(alpha_foot);
             0];

I{1}= I_foot;
% ------
% Body: Tibia
% Local frame at: RAnkle
M(2) = m1;

CoM{2} = [0;
          lc1;
         0];

I{2}= I1;
% ------
% Body: Femur
% Local frame at: RKnee 
M(3) = m2;

CoM{3} = [ 0;
          lc2;
         0];

I{3}= I2;
% ------
% Body: Hip
% Local frame at: RHip
M(4) = 0;

CoM{4} = [ 0;
          0;
         0];

I{4}= 0;
% ------
% Body: Trunk
% Local frame at: Torso 
M(5) = m_torso;

CoM{5} = [ 0;
          -lc_torso;
         0];

I{5}= I_torso;
% ------
% Body: Left femur (asigned to left hip)
% Local frame at: Lhip 
M(6) = m3;

CoM{6} = [ 0;
          -lc3;
         0];

I{6}= I3;
% ------

% Body: Left Tibia 
% Local cordinate: LKnee
M(7) = m4;

CoM{7} = [ 0;
          -lc4;
         0];

I{7}= I4;
% ------
% Body: NO BODY (it is easier to define the CoM of the foot from other frame attached to it)
% Local cordinate: LAnkle
M(8) = 0;

CoM{8} = [ 0;
          0;
         0];

I{8}= 0;
% ------
% Body: Right Foot    
% Local cordinate: LFoot Tip
M(9) = m_foot;

CoM{9} = [-lc_foot*cos(alpha_foot);
           lc_foot*sin(alpha_foot);
         0];

I{9}= I_foot;
% ------
% Body: NO BODY (asigned to sole of left foot)
% Local frame at: LSoleFoot 
M(10) = 0;

CoM{10} = [0;
          0;
         0];

I{10}= 0;
% ------

%////////////////////////////////////////////////////////////////////////

%%  Calculation of CoM in M-DH frames

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

T0 = robot.T;

% % Rotation matrices 
P = cell(1,nFrames);
for i = 1:nFrames
	P{i} = robot.Tconst(1:3,1:3,i);  % P = jR0
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

end
PI.mass = M;
PI.CoM = CoM; % ^jCoM, it is the CoM of link j w.r.t. frame j
PI.I = I; %  ^jIj i.e. is the inertia tensor of link j, located in the origin of frame j

 