function [mu, Sigma] = EKF_SLAM(s, mu, Sigma, u, z)
%% Description:
    % EKF SLAM Algorithm, predicts and filters state means & covariances.
%% Inputs:
    % s: Structure containing [list important structure features]
%% Outputs:
    % s: outputs updated filtered/predicted state mean/covariances w/ 
    %    added landmarks to state mean/covariances
%% Notes:
    % - 
%% References:
    % EKF SLAM algorithm with ML corresopences Table 10.2 pg 322,
    % Probabilistic Robotics, 2006, [Thrun et al]
    %

%% SLAM: "Come on and SLAM, if you want to jam" : Space Jame Theme Song
alpha = 0.1.*[0.9 0.2 0.3 0.4]; % robot-dependent motion noise parameters

% Pull number of landmarks, only returning it in structure for plotting
% reasons
s.nLandmarks = length(mu((s.nx+1):end))/2;
% Measurment Covariance
sigma_range = 5;
sigma_bearing = 3;
Qt = [sigma_range^2, 0;
      0, sigma_bearing^2];
% Qt = [10, 0;
%       0, 0.1];
%% Prediction Update:
    % Fx: Projection Matrix for State Update
    Fx = [eye(s.nx),zeros(s.nx,s.ny*s.nLandmarks)];
    
    % G: Jacobian of process model wrt states, V: Jacobian of process model wrt inputs
    [G, V] = Jacobian(s,mu,u,Fx); 
        
    % Define R & M?? KRAMER
    M = [(alpha(1)*abs(u(1)) + alpha(2)*abs(u(2)))^2, 0;
          0, (alpha(3)*abs(u(1)) + alpha(4)*abs(u(2)))^2];
    R = V*M*V';
    
    % Mean & Covariance Prediction
    mub = STM(s,mu,u,Fx);
    Sigmab = G*Sigma*G' + Fx'*R*Fx;
    
%% Measurement Update:
    % for each observed landmark
    num_obs_landmarks = size(z,2);
    if num_obs_landmarks > 0
        for i = 1:num_obs_landmarks
            % Initialise scratch variables
            l_idx = 0;
            
            % Initialise Matrices for each landmark
            predZ   = zeros(s.ny,s.nLandmarks+1);
            predPsi = zeros(s.ny,s.ny,s.nLandmarks+1);
            predH = zeros(s.ny,s.ny*(s.nLandmarks+1)+s.nx,s.nLandmarks+1);
            e = zeros(s.ny,s.nLandmarks+1);
            
            %  ============================== 
            if s.mahalanobis
                pi_m = zeros(1,s.nLandmarks+1); 
            else % Euclidean
                pi_e = 10*s.min_radius*ones(2,1);
            end
            % -------------------------------
            % Create a temporary landmark at observed position (in case landmark doesn't currently exist)
            mu_temp = [mub;
                       mub(1) + z(1,i)*sin(z(2,i) + mub(3));
                       mub(2) + z(1,i)*cos(z(2,i) + mub(3))];
            Sigma_temp = [Sigmab(:,:), ...
                          zeros(size(Sigmab(:,:),1),s.ny);...
                          zeros(s.ny,size(Sigmab(:,:),2) + s.ny)];
            % Initialise covariance for new landmark for new landmark
            % proportional to the range measurment squared (ie more certain on closer measurements)
            for ii = (size(Sigma_temp,1)-1):(size(Sigma_temp,1))
                 Sigma_temp(ii,ii) = z(1,i)^2/120;%(z(1,i)^1/2);%120;
            end
            
            % Compare current measurement to existing landmarks
            for j = 1:s.nLandmarks + 1
                % Predicted measurement to landmark
                [predZ(:,j),e(:,j),h] = rangeBearingModel(z(:,i), mu_temp(1:s.nx), mu_temp((s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                % Projection Matrix for State & Landmark
                Fxj = [eye(s.nx),zeros(s.nx,s.ny*j-s.ny),zeros(s.nx,s.ny),zeros(s.nx,s.ny*(s.nLandmarks+1)-s.ny*j);
                       zeros(s.ny,s.nx),zeros(s.ny,s.ny*j-s.ny),eye(s.ny),zeros(s.ny,s.ny*(s.nLandmarks+1)-s.ny*j)];
                % Projection Jacobian & Covariance update for current landmark
                predH(:,:,j) = h*Fxj;
                predPsi(:,:,j) = predH(:,:,j)*Sigma_temp*predH(:,:,j)' + Qt;
                
                % ============================== 
                if s.mahalanobis                    
                    if j <= s.nLandmarks
                        pi_m(j) = e(:,j)'*(predPsi(:,:,j)\e(:,j)); else
                        pi_m(j) = 0.02; % min mahalanobis distance to add landmark to map
                    end
                    % Track best 2 associationa:
                    if pi_m(j) < min_pi(1)
                        min_pi(2) = min_pi(1);
                        min_pi(1) = pi_m(j);
                        l_idx = j;
                    end                    
                else % Euclidean
                    ed = euclideanDistance(s,mu_temp, z(:,i), j);
                    if j < s.nLandmarks
                        if ed < pi_e(1)
                            % Track the best 2 associations
                            pi_e(2) = pi_e(1);
                            pi_e(1) = ed;
                            l_idx = j;
                        end
                    elseif pi_e(1) > s.min_radius % no landmark is identified
                        % Track the best 2 associations
                        pi_e(2) = pi_e(1);
                        pi_e(1) = ed;
                        l_idx = j;                   
                    end
                end
                % -------------------------------
            end
            % Find the best association
            if s.mahalanobis
                if (min_pi(2)/min_pi(1)) > 1.5
                    % Is it a new landmark?
                    if l_idx > s.nLandmarks
                        % Then add a landmark
                        s.nLandmarks = s.nLandmarks + 1;
                        % Update mean/covariance
                        mub = mu_temp;
                        Sigmab = Sigma_temp;
                    else
                        % Truncate H and pull best association
                        Hit = predH(:,1:s.nLandmarks*s.ny+s.nx,l_idx);
                        % Compute the kalman Gain
                        Kit = Sigmab*Hit'/predPsi(:,:,l_idx);
                        % Update mean/covariance
                        mub = mub + Kit*(e(:,l_idx));
                        Sigmab = (eye(s.nx+s.ny*s.nLandmarks)-Kit*Hit)*Sigmab;
                    end
                end
                % Collect for debugging??
                % zh(:,l_idx,t) = predZ(:,l_idx);
                % error_collect(:,l_idx,t) = e(:,l_idx);
                    
            else % Euclidean
                if l_idx > s.nLandmarks
                    % Add a landmark and expand mean/cov matrices
                    s.nLandmarks = s.nLandmarks + 1;
                    % Update mean/covariance
                    mub = mu_temp;
                    Sigmab = Sigma_temp;
                else
                    % Truncate H and pull best association
                    Hit = predH(:,1:s.nLandmarks*s.ny+s.nx,l_idx);
                    % Compute the kalman Gain
                    Kit = Sigmab*Hit'/predPsi(:,:,l_idx);
                    % Update mean/covariance
                    mub = mub + Kit*(e(:,l_idx));
                    Sigmab = (eye(s.nx+s.ny*s.nLandmarks)-Kit*Hit)*Sigmab;
                end
            end
            
        end
    end
    mu = mub;
    Sigma = Sigmab;
    
    s.s.nLandmarks = s.nLandmarks;
%% end of function
end