
global OptionContVar
if isempty(OptionContVar)  % If OptionContVar is empty we decide to chose the first option as default    
    error(['The variable "OptionContVar" is empty, you need to choose which files are used to build \n',...
           'the desired trajectories "hd(Phi)", defined in the file "OptionDesiredTrajectory.m"'],OptionContVar);
end
    
switch Case
    case 'position'    
        switch OptionContVar
            case 1
                hd = hd_Polyn(gait_parameters,t);
            case 2
                hd = hd_Polyn(gait_parameters,x);
            otherwise
                error(['The value of the variable "OptionContVar" is %d. However this option is no \n',...
                       'valid in the file "OptionDesiredTrajectory.m"'],OptionContVar);           
        end
    %  *    *     *     *      *    *     *     *     *    *     *     *        
    case 'velocity'        
        switch OptionContVar
            case 1
                Phip = [qfp; 1];
                phiDim = length(Phip) - 1; %dimension of vector "phi", where Phi = [qf^T phi^T]^T
                % terms for building matrix Jd
                dhd_Phi(:,1) = zeros(6,1); % partial derivative of "hd" w.r.t. x
                dhd_Phi(:,2) = hpd_Polyn_t(gait_parameters,t); % partial derivative of "hd" w.r.t. phi1 = t
            case 2
                Phip = qfp;
                phiDim = length(Phip) - 1; %dimension of vector "phi", where Phi = [qf^T phi^T]^T
                % terms for building matrix Jd
                dhd_Phi = dhd_Phi_Polyn(gait_parameters,x);
        end        
    %  *    *     *     *      *    *     *     *     *    *     *     *        
    case 'acceleration'
        switch OptionContVar
            case 1
                Phip = [qfp; 1];
                phiDim = length(Phip) - 1; %dimension of vector "phi", where Phi = [qf^T phi^T]^T
                % terms for building matrix "Jdp"...
                dPhi_dhd_x = zeros(6,1+phiDim);
                dPhi_dhd_t = [zeros(6,1), hppd_Polyn_t(gait_parameters,t)];
                dt_dhd_Phi = [dPhi_dhd_x*Phip, dPhi_dhd_t*Phip];
            case 2
                Phip = qfp;
                phiDim = length(Phip) - 1; %dimension of vector "phi", where Phi = [qf^T phi^T]^T
                % terms for building matrix "Jdp"...
                dPhi_dhd_x = dPhi_dhd_x_Polyn(gait_parameters,x);
                dt_dhd_Phi = [dPhi_dhd_x*Phip];
                
        end           
    %  *    *     *     *      *    *     *     *     *    *     *     *       
end