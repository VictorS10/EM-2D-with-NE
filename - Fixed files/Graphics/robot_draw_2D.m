function robot_draw_2D(robot,x0)
% x0 -> Coordinate of the support foot in X direction
%Generates a 2D-plot of the robot
T = robot.T;

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
xlabel('X');
ylabel('Y');
hold on;
axis equal
view(45,15)
grid on

rFoot = T(1:3,4,1) + [x0;0;0];
rAnkle = T(1:3,4,2) + [x0;0;0];
rKnee = T(1:3,4,3) + [x0;0;0];
rHip = T(1:3,4,4) + [x0;0;0];
Torso = T(1:3,4,5) + [x0;0;0];
lHip = T(1:3,4,6) + [x0;0;0];
lKnee = T(1:3,4,7) + [x0;0;0];
lAnkle = T(1:3,4,8) + [x0;0;0];
lFoot = T(1:3,4,9) + [x0;0;0];
lSole = T(1:3,4,10) + [x0;0;0];


% plot3([Matlab x's axis],[Matlab y's axis],[Matlab z's axis],...)
% since we want Y be the hight, we plot Y in the place for Matlab z's axis


hold on
% right foot
plot3([rFoot(1) rAnkle(1)],[rFoot(3) rAnkle(3)],[rFoot(2) rAnkle(2)],'k','LineWidth',4)
plot3([rFoot(1) rAnkle(1)],[rFoot(3) rAnkle(3)],[rFoot(2) rFoot(2)],'k','LineWidth',4)
plot3([rAnkle(1) rAnkle(1)],[rFoot(3) rAnkle(3)],[rFoot(2) rAnkle(2)],'k','LineWidth',4)
% right tibia
plot3([rAnkle(1) rKnee(1)],[rAnkle(3) rKnee(3)],[rAnkle(2) rKnee(2)],'k-o','LineWidth',4)
% right femur
plot3([rKnee(1) rHip(1)],[rKnee(3) rHip(3)],[rKnee(2) rHip(2)],'k-o','LineWidth',4)
% hip
plot3([rHip(1) lHip(1)],[rHip(3) lHip(3)],[rHip(2) lHip(2)],'g-o','LineWidth',4)
% Torso
plot3([rHip(1) Torso(1)],[Torso(3) Torso(3)],[rHip(2) Torso(2)],'g-o','LineWidth',4)
% left femur
plot3([lHip(1) lKnee(1)],[lHip(3) lKnee(3)],[lHip(2) lKnee(2)],'r-o','LineWidth',4)
% left tibia
plot3([lKnee(1) lAnkle(1)],[lKnee(3) lAnkle(3)],[lKnee(2) lAnkle(2)],'r-o','LineWidth',4)
% left foot
plot3([lAnkle(1) lFoot(1)],[lAnkle(3) lFoot(3)],[lAnkle(2) lFoot(2)],'r','LineWidth',4)
plot3([lAnkle(1) lSole(1)],[lAnkle(3) lSole(3)],[lAnkle(2) lSole(2)],'r','LineWidth',4)
plot3([lSole(1) lFoot(1)],[lSole(3) lFoot(3)],[lSole(2) lFoot(2)],'r','LineWidth',4)

% % CoM_Rotated = robot.CoM;
% % COM = surf(x/100+CoM_Rotated(1)+x0, y/100+CoM_Rotated(2)+y0, z/100+CoM_Rotated(3));
% % set(COM,'FaceColor',[0 0 0]);
% % set(COM,'EdgeColor',[0 0 0]);
% 
