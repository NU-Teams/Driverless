function [mu, Sigma] = LiDAR_update2(s, mu, Sigma, z)

sigma_range = 3e0;
sigma_bearing = 3e1*pi/180;

Qt = [sigma_range^2, 0;
     0, sigma_bearing^2];

mmd = 0.74949725; % min mahalanobis distance to add landmark to map
best_asc = 4; % Best assocition 2.1

% for each observed landmark
num_obs_landmarks = size(z,2);
% Create a list of expected landmarks and assocition variable
expected = lidarExpectedLandmarks(s,mu);
matches = [[1:num_obs_landmarks];zeros(1,num_obs_landmarks)];

% Pull number of Measurements and expected landmarks
s.nLandmarks = length(mu((s.nx+1):end))/2;
n_expected = size(expected,2);

% Seen landmarks empty vector
if s.delete
    landmark_sighted = zeros(s.nLandmarks,1);
end

if num_obs_landmarks > 0
    % Landmark management
    if s.delete
        [~,~] = landmarkManagement(mu,Sigma, 0, s, 'expected');
    end
    matches = measurementAssociation(s,z,mu,Sigma,matches,expected,num_obs_landmarks,n_expected);
    
    if n_expected > 0
%         % ------------------------------------
%         % Check measured Vs Obsereved &  Fix duplicates / mismatches
%         disp('dfgd')
%         isDuplicate = false;
%         for i = 1:n_expected
%             num_matches = length(matches(matches(2,:)==i));
%            if  num_matches > 1
%               isDuplicate = false;
%               % pull duplicate index. 
%               d_idx = matches(2,:)==i;
%               
%               
%               
%            end
%         end
        % ------------------------------------
        %
        for i = 1:num_obs_landmarks
            j = matches(2,i);
            if j > 0
                [~,e,h] = rangeBearingModel(z(:,i), mu(1:s.nx), mu((s.ny*j+s.nx-1):(s.ny*j+s.nx)));
                if e'*e < 400
                    % Projection Matrix for State & Landmark
                    Fxj = [eye(s.nx),zeros(s.nx,s.ny*j-s.ny),zeros(s.nx,s.ny),zeros(s.nx,s.ny*(s.nLandmarks)-s.ny*j);
                           zeros(s.ny,s.nx),zeros(s.ny,s.ny*j-s.ny),eye(s.ny),zeros(s.ny,s.ny*(s.nLandmarks)-s.ny*j)];
                    % Projection Jacobian & Covariance update for current landmark
                    predH = h*Fxj;
                    predPsi = predH*Sigma*predH' + Qt;

                    % Compute the kalman Gain
                    Kit = Sigma*predH'/predPsi; 
                    % Update mean/covariance
                    mu = mu + Kit*e;
                    Sigma = (eye(s.nx+s.ny*s.nLandmarks)-Kit*predH)*Sigma;
                    if s.delete
                        landmark_sighted(j) = 1;
                    end
                else
                    disp('BIG ERROR')
                end
            end
        end
        for i = 1:num_obs_landmarks
            j = matches(2,i);
            if j == 0
                if num_obs_landmarks>n_expected 
                    s.nLandmarks = s.nLandmarks + 1;
                    % Update mean/covariance
                    % -------------------------------
                    % Create a new landmark at observed position
                    mu = [mu;
                          mu(1) + z(1,i)*sin(z(2,i) + mu(3));
                          mu(2) + z(1,i)*cos(z(2,i) + mu(3))];
                    Sigma = [Sigma(:,:), ...
                             zeros(size(Sigma(:,:),1),s.ny);...
                             zeros(s.ny,size(Sigma(:,:),2) + s.ny)];

                    % Initialise covariance for new landmark for new landmark
                    % proportional to the range measurment squared (ie more certain on closer measurements)
                    for ii = (size(Sigma,1)-1):(size(Sigma,1))
                         Sigma(ii,ii) = 2.5e1; %z(1,i)^3/((0.1*length(z)*z(1,i))^2);%(z(1,i)^1/2);%120;
                    end

                    % Increase the vector size
                    if s.delete
                        landmark_sighted = [landmark_sighted; 1];
                    end
                else
                    mu = [mu;
                          mu(1) + z(1,i)*sin(z(2,i) + mu(3));
                          mu(2) + z(1,i)*cos(z(2,i) + mu(3))];
                    Sigma = [Sigma(:,:), ...
                             zeros(size(Sigma(:,:),1),s.ny);...
                             zeros(s.ny,size(Sigma(:,:),2) + s.ny)];

                    % Initialise covariance for new landmark for new landmark
                    % proportional to the range measurment squared (ie more certain on closer measurements)
                    for ii = (size(Sigma,1)-1):(size(Sigma,1))
                         Sigma(ii,ii) = 2.5e1; %z(1,i)^3/((0.1*length(z)*z(1,i))^2);%(z(1,i)^1/2);%120;
                    end

                    % Increase the vector size
                    if s.delete
                        landmark_sighted = [landmark_sighted; 1];
                    end
                    
                end
            end
        end
    end
    % If zero landmarks, then we mark all a new landmark/s
    if s.nLandmarks == 0
        for i = 1:num_obs_landmarks
            % ------------------------------------
            % Check to add a landmark
            % Add a landmark and expand mean/cov matrices
            s.nLandmarks = s.nLandmarks + 1;
            % Update mean/covariance
            % -------------------------------
            % Create a new landmark at observed position
            mu = [mu;
                  mu(1) + z(1,i)*sin(z(2,i) + mu(3));
                  mu(2) + z(1,i)*cos(z(2,i) + mu(3))];
            Sigma = [Sigma(:,:), ...
                     zeros(size(Sigma(:,:),1),s.ny);...
                     zeros(s.ny,size(Sigma(:,:),2) + s.ny)];

            % Initialise covariance for new landmark for new landmark
            % proportional to the range measurment squared (ie more certain on closer measurements)
            for ii = (size(Sigma,1)-1):(size(Sigma,1))
                 Sigma(ii,ii) = 2.5e1; %z(1,i)^3/((0.1*length(z)*z(1,i))^2);%(z(1,i)^1/2);%120;
            end

            % Increase the vector size
            if s.delete
                landmark_sighted = [landmark_sighted; 1];
            end
        end
%     elseif expected(expected(2,:)==0) % Determine measurements vs expected
%         if num_obs_landmarks>n_expected 
%             for i = 1:(num_obs_landmarks-num_obs_landmarks)
% 
%             end
%         end
%     else % detemine possibility of new landmark
    end
    % ------------------------------------
    % Do Measurment update
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