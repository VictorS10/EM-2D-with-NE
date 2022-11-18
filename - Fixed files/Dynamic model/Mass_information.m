% If you want to fold the code for each section in YOUR MATLAB go to
%    Editor/Debugger (MATLAB Preferences) > Code Folding and enable "Sections" by checking on it
function PI = Mass_information
%% Version of the  18 December 2017
% Information for dynamic parameters of the robot

%% I.   Masses - Center of Masses - Inertia Tensors
%  ============================================
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

M = zeros(9,1);

% CoM = [X_G ; Y_G ; Z_G]; kg/mÂ²
%
% I= [I_{xx} , I_{xy} , I_{xz} ;
%     I_{yx} , I_{yy} , I_{yz} ;
%     I_{zx} , I_{zy} , I_{zz}]; kg/mÂ²

%% CoMs and Masses   
% Body: Right Foot    
% Local cordinate: RFoot Tip
M(1) = 1;

CoM{1} = [ -0.1;
          0.1;
         0];

I{1}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Tibia
% Local cordinate: RAnklePitch 
M(2) = 1;

CoM{2} = [ 0;
          0.5;
         0];

I{2}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Femur
% Local cordinate: RKneePitch 
M(3) = 1;

CoM{3} = [ 0;
          0.5;
         0];

I{3}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Trunk
% Local cordinate: HipPitch 
M(4) = 1;

CoM{4} = [ 0;
          0.5;
         0];

I{4}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Left femur
% Local cordinate: LKneePitch
M(5) = 1;

CoM{5} = [ 0;
          0.5;
         0];

I{5}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Left Tibia
% Local cordinate: LAnklePitch
M(6) = 1;

CoM{6} = [ 0;
          0.5;
         0];

I{6}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------
% Body: Right Foot    
% Local cordinate: LFoot Tip
M(7) = 1;

CoM{7} = [ -0.1;
          0.1;
         0];

I{7}= [1, 0, 0;
       0, 1, 0;
       0, 0, 1];
% ------

%////////////////////////////////////////////////////////////////////////

%% III. Transformation matrices w.r.t frame 0 (Tip of the RFoot)
joints = 6;
q = zeros(joints,1); 
qD = zeros(joints,1);
robot = struct('joints',joints,'q',q,'qD',qD);
T0=DGM(robot);
  
%% IV.  Calculation of I. in M-DH frames

% Base change I_j = inv(P_j) I_j_s P_j

% Transport I_j = I_j_s + M Y_s
% with Y_s = [b^2+c^2,-a*b,-a*c;
%             -a*b,c^2+a^2,-b*c;
%             -a*c,-b*c,a^2+b^2]:
% Knowing CoM = [a;b;c];

% Transport of the CoM 
% d_xg = d_x - d_x_0
% d_yg = d_y - d_y_0
% d_zg = d_z - d_z_0

% % Rotation matrices 
P = cell(1,9);
for i = 1:9
	P{i} = inv(T0(1:3,1:3,i));  % P = jR0
end

% % Translation of the frames
vec_C = cell(1,9);
for i = 1:9
    vec_C{i} = [0;0;0];
end
 % vec_C que creo que son
 % Es un vector de traslacion para que los marcos de los centros de masa
 % coincidan con los marcos DH. Solo son 4 marcos los que no coinciden y
 % necesitan una traslacion:
    vec_C{1}=[0.095;0;-0.046];
    vec_C{3}=[0;0;-0.1];
    vec_C{4}=[0;0;-0.1];
    vec_C{7} = [0;0;-0.0350];     %to translate to the Torso
    
for i = 1 : 9
    C = vec_C{i}; %vec_C contient les vecteurs de translation des repères de références;
    inter = CoM{i};
    tmp=[inter(1)-C(1);inter(2)-C(2);inter(3)-C(3)];
    CoM{i} = P{i}*tmp; % calcul de la position du centre de masse dans les bon repère de référence 

    Y = [tmp(2)^2+tmp(3)^2,-tmp(1)*tmp(2),-tmp(1)*tmp(3);
     -tmp(1)*tmp(2),tmp(3)^2+tmp(1)^2,-tmp(2)*tmp(3);
     -tmp(1)*tmp(3),-tmp(2)*tmp(3),tmp(1)^2+tmp(2)^2];

    I{i} = I{i} + M(i) * (Y);   % application of translation  % SIj = bIb + Mj*Yj  "S" is the Aldebaran frame 
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
    for i = 1:9
        I{i} = alphaM*I{i};
        MassTorso = MassTorso + (1-alphaM)*M(i);
        M(i) = alphaM*M(i);
    end
    
    % % Assignment of the masss to the frame 15 (hip),
    M(7)= M(7) + MassTorso;% Rest of the mass asigned to frame 15
    % If less than 20% of the mass is concentrated in the frame 15 the same CoM position (of frame 15) will be used, but...
    if alphaM < 0.75
        CoM{7} = [0; -.05; -.15]; % we change the CoM position of frame 15 if more than 20% of the mass are concentred in this frame...
    end
end
% % ==============================================================================================
PI.masse = M;
%Para pruebas con jacobianos
% for i=1:27
%     CoM{i}=zeros(3,1);
% end
PI.CoM = CoM; % ^jCoM, it is the CoM of link j w.r.t. frame j
PI.I = I; %  ^jIj i.e. is the inertia tensor of link j, located in the origin of frame j

 