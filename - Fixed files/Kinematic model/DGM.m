function T = DGM(robot)
%% computing of the geometric model (transformations matrix) of the robot
nFrames = 10; % 6 for each joint, and + 4, tip foot 1, Torso, tip and sole foot 2
T=zeros(4,4,nFrames); 
d1 = 0.2; % horizontal distance from the foot sole (below the ankle) to its tip
d2 = 0.1;   % vertical distance from the foot sole (below the ankle) to its ankle
dd = sqrt(d1^2 + d2^2);
dAngle = atan2(d2,d1);
l1 = 1; % Lenght of the tibia
l2 = 1; % Lenght of the femur
d3 = 1; %length of torso
d4 = 0.3; % horizontal length of the hip

% Description of the robot structure
% ------------------------------------
% The system is composed of n+1 links, denoted L0, �, Ln, and n joints. The link L0 is
% the base of the robot while link Ln is the terminal link. Joint j connects link j with link j-1
% (See Symoro book from Wisama KHALIL 2003)
% The frame j coordinates, which is fixed with link j, is defined such that:
% - the zj axis is along the axis of joint j,
% - the xj axis is along the common normal between zj and zj+1. If the axes zj and zj+1 are
% parallel, the choice of xj is not unique.
% - the origin Oj is the intersection of zj and xj.
% DH parameters:
% alphaj: angle between zj-1 and zj around xj-1
% dj: distance between zj-1 and zj along xj-1,
% thetaj: angle between xj-1 and xj around zj,
% rj: distance between xj-1 and xj along zj
% NOTE: In a planar robot "r" and "alpha" are always 0

% When xi is not along the common normal between zi and zj, "i" is the antecedent frame
% we define uj along the common normal between zi and zj, and the parameters are:
% gammaj: angle between xi and uj about zi
% bj: distance between xi and uj along zi
% alphaj: angle between zi and zj about uj
% dj: distance between zi and zj along uj
% thetaj: angle between uj and xj about zj
% rj: distance between uj and xj along zj
% NOTE: In a planar robot "gamma", "b", "alpha" and "r" are always 0

gamma = zeros(nFrames,1);
b = zeros(nFrames,1);
alpha = zeros(nFrames,1);
r = zeros(nFrames,1);
r(5:6) = -d4/2;


theta = [pi-dAngle,... 
    -pi/2 + dAngle + robot.q(1),...
    robot.q(2),...
    robot.q(3),...
     -pi,... 
    robot.q(4),...
    robot.q(5),...
    pi/2 - dAngle + robot.q(6),...
    pi + dAngle,...
    pi]; % 
% d = [d1;dd;l1;l2;3*d3;d3;l2;l1;dd];
d = [d1;dd;l1;l2;d3;d3;l2;l1;dd;d1];
iT_j = cell(1,10);

% Antecedent frame to current frames
ant = robot.ant;

% In this case, since there is only one branch, we can use any the next: 
for j=1:nFrames
    % Computing chain structure, (we don't need "gamma" and "b" parametres)
%     iT_j{j} = [cos(theta(j)) -sin(theta(j)) 0 d(j);
%         cos(alpha(j))*sin(theta(j)) cos(alpha(j))*cos(theta(j)) -sin(alpha(j)) -r(j)*sin(alpha(j));
%         sin(alpha(j))*sin(theta(j)) sin(alpha(j))*cos(theta(j)) cos(alpha(j)) r(j)*cos(alpha(j));
%         0 0 0 1];
    
    % Computing Tree structure, (we need "gamma" and "b" parametres)
    iT_j{j} = [cos(gamma(j))*cos(theta(j))-sin(gamma(j))*cos(alpha(j))*sin(theta(j)),...
        -cos(gamma(j))*sin(theta(j))-sin(gamma(j))*cos(alpha(j))*cos(theta(j)),...
        sin(gamma(j))*sin(alpha(j)),...
        d(j)*cos(gamma(j))+r(j)*sin(gamma(j))*sin(alpha(j));
        sin(gamma(j))*cos(theta(j))+cos(gamma(j))*cos(alpha(j))*sin(theta(j)),...
        -sin(gamma(j))*sin(theta(j))+cos(gamma(j))*cos(alpha(j))*cos(theta(j)),...
        -cos(gamma(j))*sin(alpha(j)),...
        d(j)*sin(gamma(j))-r(j)*cos(gamma(j))*sin(alpha(j));
        sin(alpha(j))*sin(theta(j)), sin(alpha(j))*cos(theta(j)), cos(alpha(j)), r(j)*cos(alpha(j))+b(j);
        0,0,0,1];
end

% In this case ant(j) is always j-1
T(:,:,1) = iT_j{1};
for j = 2 : nFrames
    T(:,:,j) = T(:,:,ant(j)) * iT_j{j};
end

