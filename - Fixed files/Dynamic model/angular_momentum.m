function [L LD] = angular_momentum(robot)


T = robot.T;
nFrames = robot.nFrames;
CoM_j = robot.PI.CoM; % Center of mass of each link refered to its joint
M_j = robot.PI.mass;
I_j = robot.PI.I;
qD = [0.1,0.2,0.3,0.3,0.2,0.1];
qDD = [0,0,0,0,0,0];
robot.qD = qD;
act = robot.act;

[Vel_CoM, Vel_CoMs, Ace_CoM, Ace_CoMs, AceFrame_i] = VelAceCoMs_Frames(robot,qDD);

L_j = zeros(3,nFrames);
LD_j = zeros(3,nFrames);
i=1;
for j=1:nFrames
    if act(j)
        omega = qD(i);
        omegaD = qD(i);
        i=i+1;
    else
        omega = 0;
        omegaD = 0;
        
    end
    L_j(:,j) = I_j{j}*[0;0;omega] +  M_j(j)*cross_matrix(T(1:3,:,j)*[CoM_j{j};1])*Vel_CoMs{j};
    LD_j(:,j) = I_j{j}*[0;0;omegaD] +  M_j(j)*(cross_matrix(Vel_CoMs{j})*Vel_CoMs{j}...
        + cross_matrix(T(1:3,:,j)*[CoM_j{j};1])*Ace_CoMs{j});   
end

L = sum(L_j,2);
LD = sum(LD_j,2);


Reference frames must be around the CoM