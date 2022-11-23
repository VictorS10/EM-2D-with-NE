function robot_draw_2D(robot,x0,frames)
% x0 -> Coordinate of the support foot in X direction
%Generates a 2D-plot of the robot

if nargin == 2
    frames = false;
end

global coms
% % % ---------------------------------------------------------------------------  
% % supportFrame_Rotated = RotZ_01'*[x0;y0];
% % distCoM = robot.CoM(1); 
% % suppX = supportFrame_Rotated(1);
% % axis([cos(alpha)*(distCoM+suppX)-0.7, cos(alpha)*(distCoM+suppX)+0.7,...
% %       sin(alpha)*(distCoM+suppX)-0.7, sin(alpha)*(distCoM+suppX)+0.7, 0 1.4]);
% % % ---------------------------------------------------------------------------  
% global GlobalCoMX GlobalCoMY
% if ~isempty(GlobalCoMX) && ~isempty(GlobalCoMX)
%     axis([GlobalCoMX-0.3, GlobalCoMX+0.3,GlobalCoMY-0.3, GlobalCoMY+0.3, 0 0.6]);
% else
%     axis([-0.4 0.4 -0.4 0.4 -0.1 0.4]);  % For a fixed plot %*
% end
% % ---------------------------------------------------------------------------  
xlabel('x');
ylabel('z');
zlabel('y');
hold on;
axis equal
view(45,15)
grid on

% Rotate all frames to plot it in MATLAB
T_matlab = [1 0 0 0;
            0 0 -1 0;
            0 1 0 0;
            0 0 0 1];
T = robot.T; 
for i=1:10
     T(1:3,4,i) = T(1:3,4,i) + [x0;0;0];
    T(:,:,i) = T_matlab*T(:,:,i);
end
% Now T has the direction for plotting in matlab JUST IN THIS FILE

rFoot = T(1:3,4,1);
rAnkle = T(1:3,4,2);
rKnee = T(1:3,4,3);
rHip = T(1:3,4,4);
Torso = T(1:3,4,5);
lHip = T(1:3,4,6);
lKnee = T(1:3,4,7);
lAnkle = T(1:3,4,8);
lFoot = T(1:3,4,9);
lSole = T(1:3,4,10);

nMasses = length(find(robot.PI.mass)); %bumber of masses of the robot
mass = robot.PI.mass;
CoM_j = robot.PI.CoM;
CoM_Matlab = zeros(3,nMasses);
nFrames = robot.nFrames;
i=1;
for j=1:nFrames
    if mass(j)
        CoM_Matlab(:,i) = T(1:3,:,j) * [CoM_j{j};1]; % This is the vector position of CoM_j w.r.t. frame 0
        i=i+1;
    end
end

% Plot
hold on
% right foot
plot3([rFoot(1) rAnkle(1)],[rFoot(2) rAnkle(2)],[rFoot(3) rAnkle(3)],'k','LineWidth',4)
plot3([rFoot(1) rAnkle(1)],[rFoot(2) rAnkle(2)],[rFoot(3) rFoot(3)],'k','LineWidth',4)
plot3([rAnkle(1) rAnkle(1)],[rFoot(2) rAnkle(2)],[rFoot(3) rAnkle(3)],'k','LineWidth',4)
% right tibia
plot3([rAnkle(1) rKnee(1)],[rAnkle(2) rKnee(2)],[rAnkle(3) rKnee(3)],'k-o','LineWidth',4)
% right femur
plot3([rKnee(1) rHip(1)],[rKnee(2) rHip(2)],[rKnee(3) rHip(3)],'k-o','LineWidth',4)
% hip
plot3([rHip(1) lHip(1)],[rHip(2) lHip(2)],[rHip(3) lHip(3)],'g-o','LineWidth',4)
% Torso
plot3([rHip(1) Torso(1)],[Torso(2) Torso(2)],[rHip(3) Torso(3)],'g-o','LineWidth',4)
% left femur
plot3([lHip(1) lKnee(1)],[lHip(2) lKnee(2)],[lHip(3) lKnee(3)],'r-o','LineWidth',4)
% left tibia
plot3([lKnee(1) lAnkle(1)],[lKnee(2) lAnkle(2)],[lKnee(3) lAnkle(3)],'r-o','LineWidth',4)
% left foot
plot3([lAnkle(1) lFoot(1)],[lAnkle(2) lFoot(2)],[lAnkle(3) lFoot(3)],'r','LineWidth',4)
plot3([lAnkle(1) lSole(1)],[lAnkle(2) lSole(2)],[lAnkle(3) lSole(3)],'r','LineWidth',4)
plot3([lSole(1) lFoot(1)],[lSole(2) lFoot(2)],[lSole(3) lFoot(3)],'r','LineWidth',4)

% Plot CoM of each link
for i=1:nMasses
    plot3(CoM_Matlab(1,i),CoM_Matlab(2,i),CoM_Matlab(3,i),'bo','LineWidth',4)
end
% Plot CoM of the whole body
CoM_Body_Matlab=T_matlab(1:3,1:3)*(robot.CoM + [x0;0;0]);
plot3(CoM_Body_Matlab(1),CoM_Body_Matlab(2),CoM_Body_Matlab(3),'co','LineWidth',6)


if frames
    for i=1:10
        frame(T(:,:,i),'c',.05,0)
    end
end

    
% % CoM_Rotated = robot.CoM;
% % COM = surf(x/100+CoM_Rotated(1)+x0, y/100+CoM_Rotated(2)+y0, z/100+CoM_Rotated(3));
% % set(COM,'FaceColor',[0 0 0]);
% % set(COM,'EdgeColor',[0 0 0]);
% 
