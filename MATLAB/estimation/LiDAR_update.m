function [mu, Sigma] = LiDAR_update(s, mu, Sigma, z)

sigma_range = 5e-1;
sigma_bearing = 5*pi/180;


mmd = 2.74949725; % min mahalanobis distance to add landmark to map
best_asc = 2.1e0; % Best assocition 2.1
s.nLandmarks = length(mu((s.nx+1):end))/2;
% Seen landmarks empty vector
if s.delete
    landmark_sighted = zeros(s.nLandmarks,1);
end
% for each observed landmark
num_obs_landmarks = size(z,2);
if num_obs_landmarks > 0
    % Landmark management
    if s.delete
        [~,~] = landmarkManagement(mu,Sigma, 0, s, 'expected');
    end
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
            min_pi = 1000*mmd*ones(2,1);
        else % Euclidean
            pi_e = 10*s.min_radius*ones(2,1);
        end
        % -------------------------------
        % Create a temporary landmark at observed position (in case landmark doesn't currently exist)
        mu_temp = [mu;
                   mu(1) + z(1,i)*sin(z(2,i) + mu(3));
                   mu(2) + z(1,i)*cos(z(2,i) + mu(3))];
        Sigma_temp = [Sigma(:,:), ...
                      zeros(size(Sigma(:,:),1),s.ny);...
                      zeros(s.ny,size(Sigma(:,:),2) + s.ny)];
        % Initialise covariance for new landmark for new landmark
        % proportional to the range measurment squared (ie more certain on closer measurements)
        for ii = (size(Sigma_temp,1)-1):(size(Sigma_temp,1))
             Sigma_temp(ii,ii) = z(1,i)^3/((0.1*length(z)*z(1,i))^2);%(z(1,i)^1/2);%120;
        end
%             Qt(1,1) = z(1,i)^2/(0.2*length(z));
%             Qt(2,2) = sigma_bearing^2/(length(z));


        % Compare current measurement to existing landmarks
        for j = 1:s.nLandmarks + 1
            Qt = [sigma_range^2, 0;
                 0, sigma_bearing^2];
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
                    pi_m(j) = mmd; % min mahalanobis distance to add landmark to map
                end
                % Track best 2 associationa:
                if pi_m(j) < min_pi(1)
                    min_pi(2) = min_pi(1);
                    min_pi(1) = pi_m(j);
                    l_idx = j;
                elseif pi_m(j) < min_pi(2)
                    min_pi(2) = pi_m(j);
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
            if (min_pi(2)/min_pi(1)) > best_asc
                % Is it a new landmark?
                if l_idx > s.nLandmarks  || landmark_sighted(l_idx)
                    % Then add a landmark
                    s.nLandmarks = s.nLandmarks + 1;
                    % Update mean/covariance
                    mu = mu_temp;
                    Sigma = Sigma_temp;
                    % Increase the vector size
                    if s.delete
                        landmark_sighted = [landmark_sighted; 1];
                    end
                else
                    % Truncate H and pull best association
                    Hit = predH(:,1:s.nLandmarks*s.ny+s.nx,l_idx);
                    % Compute the kalman Gain
                    Kit = Sigma*Hit'/predPsi(:,:,l_idx);
                    % Update mean/covariance
                    mu = mu + Kit*(e(:,l_idx));
                    Sigma = (eye(s.nx+s.ny*s.nLandmarks)-Kit*Hit)*Sigma;
                    % Mark landmark seen
                    if s.delete
                        landmark_sighted(l_idx) = 1;
                    end
                end
            else
%                     disp('ignored')
%                     disp((min_pi(2)/min_pi(1)))
            end
            % Collect for debugging??
            % zh(:,l_idx,t) = predZ(:,l_idx);
            % error_collect(:,l_idx,t) = e(:,l_idx);

        else % Euclidean
            if l_idx > s.nLandmarks || landmark_sighted(l_idx)
                % Add a landmark and expand mean/cov matrices
                s.nLandmarks = s.nLandmarks + 1;
                % Update mean/covariance
                mu = mu_temp;
                Sigma = Sigma_temp;
                % Increase the vector size
                if s.delete
                    landmark_sighted = [landmark_sighted; 1];
                end
            else
                % Truncate H and pull best association
                Hit = predH(:,1:s.nLandmarks*s.ny+s.nx,l_idx);
                % Compute the kalman Gain
                Kit = Sigma*Hit'/predPsi(:,:,l_idx);
                % Update mean/covariance
                mu = mu + Kit*(e(:,l_idx));
                Sigma = (eye(s.nx+s.ny*s.nLandmarks)-Kit*Hit)*Sigma;
                % Mark landmark seen
                if s.delete
                    landmark_sighted(l_idx) = 1;
                end
            end
        end

    end
end
% 
% mu = mu;
% Sigma = Sigma;
% Manage Landmarks
if s.delete
    [mu,Sigma] = landmarkManagement(mu,Sigma,landmark_sighted, s, 'delete');
end


%% end of funtion
end