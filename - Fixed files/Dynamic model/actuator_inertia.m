function [ Im ] = actuator_inertia
%Motor inertias for the 2d biped robot
%Taking into account the rotor inertia and the reudction ratio

%Rotor inertia
Ir = [1;...q1
    1;...q2
    1;...q3
    1;...q4
    1;...q5
    1];...q6
 
Ir = Ir*1e-7;

%Reduction ratios
N = [1;...q1
    1;...q2
    1;...q3
    1;...q4
    1;...q5
    1];...q6
    
Im = Ir.*N.^2;

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
    Im=alphaM*Im;
end

