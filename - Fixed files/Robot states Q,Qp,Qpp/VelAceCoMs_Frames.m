function [Vel_CoM, Vel_CoMs, Ace_CoM, Ace_CoMs, AceFrame_i] = VelAceCoMs_Frames(robot,qpp)
% Cálculo de las velocidades y aceleraciones de cada Marco y del centro de masa usando el método recursivo
% -------------------------------------------------------------------------------------------
% crossM = robot.crossM;  % Todos los vectores ^0a_j en forma de matriz antisimétrica para poder realizar el producto vectorial
                          % como producto matricial.
ant = robot.ant; % Marcos antecedentes
act = robot.act;  % Articulaciones actuadas en cada marco
nFrames = robot.nFrames;
M = robot.PI.mass; % Masas adjuntas a cada marco
T = robot.T;
CoM_j = robot.PI.CoM;
qp = robot.qD;

% Marcos antecedentes al marco Actual:
% -------------------------------------------------
% numMarco   1 2 3 4 5 6 7 8 9 10 
% ant    = [ 0,1,2,3,4,5,6,7,8, 9];
% Marcos actuados, se pone cero si no es actuado y se pune un numero consecutivo para definir el numero de articulación:
% --------------------------------------------------------------------------------------------------------------------------
% numMarco   1 2 3 4 5 6 7 8 9 10 
% act    = [ 0,1,2,3,0,4,5,6,0,0];
% Observando la info anterior podemos agrupar los MARCOS que tienen ARTICULACION
% --------------------------------------------------------------------------------------------------------------------------
% i = 1 2 3 4 5 6
% Fr = [2,3,4,6,7,8,];
% Estos Marcos son los que se utilizan para calcular las Jacobianas (only the ones that have joints/actuarors)

P = cell(1,nFrames);   % Posicion del marco i desde i-1
Wi_1 = cell(1,nFrames);  % Velocidad angular del marco i-1 Siempre respecto al marco i-1
Vi_1 = cell(1,nFrames);  % Velocidad lineal del marco i-1 Siempre respecto al marco i-1
WPi_1 = cell(1,nFrames);  % Aceleracion angular del marco i-1 Siempre respecto al marco i-1
VPi_1 = cell(1,nFrames);   % Aceleracion lineal del marco i-1 Siempre respecto al marco i-1
Wi = cell(1,nFrames);  % Velocidad angular del marco i 
Vi = cell(1,nFrames);  % Velocidad lineal del marco i 
V_Gi = cell(1,nFrames);  % Velocidad lineal del CoM unido al marco i 
WPi = cell(1,nFrames);  % Aceleracion angular del marco i
VPi = cell(1,nFrames);   % Aceleracion lineal del marco i 
Vp_Gi = cell(1,nFrames);  % Aceleración lineal del CoM unido al marco i 
AceFrame_i = cell(1,nFrames);  % Aceleración (lineal y angular) del Marco i 

% Inicialización
P{1} = T(1:3,4,1);
Wi_1{1} = [0;0;0];
Vi_1{1} = [0;0;0];
WPi_1{1} = [0;0;0];
VPi_1{1} = [0;0;0];

Vel_CoMs = cell(1,nFrames);  % Velocidad lineal de los CoMs unido al marco i respecto al marco 0
Vel_CoMs{1} = [0;0;0];
Vel_CoM = [0;0;0];
Ace_CoMs = cell(1,nFrames);  % Velocidad lineal de los CoMs unido al marco i respecto al marco 0
Ace_CoMs{1} = [0;0;0];
Ace_CoM = [0;0;0];
cont = 1;
for i = 2 : nFrames  % Se empieza desde el marco 2 ya que no existe actuador en el primer marco 
    % IMPORTANT: Note that "ant(i)" is the previous frame of "i". So in a serial chain "ant(i) = i-1", however in a three
    %      chain we must use "ant(i)" because in a new branch we need to re-take ITS previos frame, for example in our robot
    %       the previous frame of 15 is 7 (not 14)... However in order to make some explanations clearer "i-1" was used =P
    % -----------------------
    i_1Ti = inv(T(1:4,1:4,ant(i)))*T(1:4,1:4,i); % ^{i-1}T_{i} = ^{i-1}T_0 * ^0T_{i}
    iRi_1 = inv(T(1:3,1:3,i))*T(1:3,1:3,ant(i)); % ^iR_{i-1} = ^iR_0 * ^0R_{i-1}
    Pp = i_1Ti(1:3,4);  % ^{i-1}P_{i-1,i}
    P{i} = iRi_1*Pp;        
    Wi{ant(i)} = iRi_1*Wi_1{ant(i)};   %^iw_{i-1} = ^iR_{i-1} * ^{i-1}w_{i-1}   % Velocidad angular del marco i-1 (respecto a i)
    Vi{ant(i)} = iRi_1*Vi_1{ant(i)};   %^iv_{i-1} = ^iR_{i-1} * ^{i-1}v_{i-1}   % Velocidad lineal del marco i-1 (respecto a i)
    WPi{ant(i)} = iRi_1*WPi_1{ant(i)};   %^iwp_{i-1} = ^iR_{i-1} * ^{i-1}wp_{i-1}   % Aceleracion angular del marco i-1 (respecto a i)
    VPi{ant(i)} = iRi_1*VPi_1{ant(i)};   %^ivp_{i-1} = ^iR_{i-1} * ^{i-1}vp_{i-1}   % Aceleracion lineal del marco i-1 (respecto a i)
    % -------------------
    iR0 = inv(T(1:3,1:3,i));
    ai = iR0*T(1:3,3,i); % ^ia = ^iR_0 ^0a
    P_Gi = CoM_j{i}; 
    % if M(i) == 0   % It is asigned the same velocity that the past frame
    if act(i) == 0 % Ya que NO existe actuador en los marcos que NO tienen asignada una massa.    
        Wi{i} = Wi{ant(i)};
        Vi{i} = Vi{ant(i)}; 
        V_Gi{i} = [0;0;0];
        WPi{i} = WPi{ant(i)};
        VPi{i} = VPi{ant(i)};
        Vp_Gi{i} = [0;0;0];
    else        
        Wi{i} = Wi{ant(i)} + ai*qp(cont);
        Vi{i} = Vi{ant(i)} + cross_matrix(Wi{ant(i)})*P{i};
        V_Gi{i} = Vi{i} + cross_matrix(Wi{i})*P_Gi;
        WPi{i} =  WPi{ant(i)} + ai*qpp(cont) + cross_matrix(Wi{i})*ai*qp(cont);
        VPi{i} = VPi{ant(i)} + cross_matrix(WPi{ant(i)})*P{i} + cross_matrix(Wi{ant(i)})*cross_matrix(Wi{ant(i)})*P{i};
        Vp_Gi{i} = VPi{i} + cross_matrix(WPi{i})*P_Gi + cross_matrix(Wi{i})*cross_matrix(Wi{i})*P_Gi;
        cont = cont + 1;
    end
    % ---------------------
    % Éstos serán las nuevas velocidades y aceleraciones:
    Wi_1{i} = Wi{i};    % ^{i-1}w_{i-1}
    Vi_1{i} = Vi{i};    % ^{i-1}v_{i-1}
    WPi_1{i} = WPi{i};  % ^{i-1}wp_{i-1}
    VPi_1{i} = VPi{i};  % ^{i-1}vp_{i-1}
    % ---------------------
    Vel_CoMs{i} = T(1:3,:,i)*[V_Gi{i};1];  % ^0v_G = ^0T_i*^iv_G CREO QUE AQUI DEBE SER 1 (ya lo cambié)
    Vel_CoM = Vel_CoM +  M(i)*Vel_CoMs{i};
    Ace_CoMs{i} = T(1:3,:,i)*[Vp_Gi{i};1];  % ^0vp_G = ^0T_i*^ivp_G  CREO QUE AQUI DEBE SER 1
    Ace_CoM = Ace_CoM +  M(i)*Ace_CoMs{i};
    % Formamos el vector de aceleraciones (linear y angular) de cada uno de los Marcos de referencia
    Ace_i = T(1:3,:,i)*[VPi{i};1];  % ^0vp_i = ^0T_i*^ivp_i  CREO QUE AQUI DEBE SER 1
    WP_i = T(1:3,:,i)*[WPi{i};1];   % ^0wp_i = ^0T_i*^iwp_i  CREO QUE AQUI DEBE SER 1
    AceFrame_i{i} = [Ace_i; WP_i];
end

Vel_CoM = Vel_CoM/robot.mass;
Ace_CoM = Ace_CoM/robot.mass;
