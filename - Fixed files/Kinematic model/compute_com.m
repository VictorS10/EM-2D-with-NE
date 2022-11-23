function [CoM,J_CoM,J_LSole,crossM,J_CoMs] = compute_com(robot,PI)
nFrames = robot.nFrames;
%% CoM position/velocity

%Mass informations
M = PI.mass;
CoM_j = PI.CoM;
%///////////////////////////////////////////////////////////////////////////
%Initialisations
CoM = [0;0;0];
J_CoMs = zeros(3,robot.joints,nFrames);
J_CoM = zeros(3,robot.joints);
T = robot.T;
ant = robot.ant;
act = robot.act;
crossM = zeros(3,3,nFrames);

%Computation of the cross matrices
for i = 1:nFrames
crossM(:,:,i) = cross_matrix(T(1:3,3,i)); % It transforms vector "a" of the transformation matrix
                                            % T = [s n a p; 0 0 0 1]; into an sknew-symmetric matrix
end

for j = 1 : nFrames
    if M(j) ~=0
        % Center of mass position (X,Y)
        MS_j = T(1:3,:,j) * [CoM_j{j};1]; % This is the vector position of CoM_j w.r.t. frame 0
        CoM = CoM + M(j) * MS_j;         % The global CoM of the robot is computed "Sum(j=1:nFrames) m* ^0Com_j"
        %Center of mass velocity (XD,YD)
        J_X = zeros(3,robot.joints);      % A 3xnFrames matrix is created to compute J_CoM_j
        F = j;% The jacobians for each frame will be created (for the ones with no mass assigned, will be a zero matrix)
        while F~=1    % ¿Are we in Frame 1? If it is, we're going out of while loop, if not, we continue... (F is never less than 1)
            % Here, each column of J_CoM_j matri is computed. Note that it "jumps"  the columns that
            % correspondns to the frames were NO mass was assigned.. Note also that the jabobians are been 
            % created backward, i.e. form the end to the beginig
            if act(F)~=0 % ¿Does frame F has a joint (q)? if does, it enters the "if", if doesn't then no =P
               J_X(:,act(F)) =crossM(:,:,F)*(MS_j-T(1:3,4,F)); %col  -> 0a_j X (0Pcom_j - 0P_j)
            end
            F = ant(F);
        end
        J_CoMs(:,:,j)=J_X;               % This es Jacobian J_CoM_j
        J_CoM = J_CoM+M(j)*J_X;          % The global Jacobian J_CoM is also computed (almost)
     end
end

CoM = CoM / robot.mass;   % = 1/mT Sum_{i=1}^n a_j X ^jCom_j   where  jCom_j is the constante CoM vector of link j 
                          % w.r.t. frame j, and a_j is the vector the "snap" matrix of frame j w.r.t. 0
J_CoM = J_CoM/robot.mass; % The global Jacobian J_CoM is finally computed 

%% J_Lsole is the jacobian of the frame located on the sole of free (left) foot (below the ankle)
% This is useful to compute the work velocities of this free foot
F = 10;
J_LSole = zeros(3,robot.joints);
while F~=1
        if act(F)~=0 
            J = crossM(:,:,F)*(T(1:3,4,10)-T(1:3,4,F));
            J_LSole(:,act(F)) = J(:);
        end
        F = ant(F);
end







