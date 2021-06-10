function [matches] = measurementAssociation(s,z,mu,Sigma,matches,expected,num_obs_landmarks,n_expected)
sigma_range = 3e0;
sigma_bearing = 3e0*pi/180;

Qt = [sigma_range^2, 0;
     0, sigma_bearing^2];

mmd = 0.74949725; % min mahalanobis distance to add landmark to map
best_asc = 2.1; % Best assocition 2.1
for i = 1:num_obs_landmarks
    % Initialise scratch variables
    l_idx = 0;

    % Initialise Matrices for each landmark
    predZ   = zeros(s.ny,n_expected+1);
    predPsi = zeros(s.ny,s.ny,n_expected+1);
    predH = zeros(s.ny,s.ny*(s.nLandmarks)+s.nx,n_expected+1);
    e = zeros(s.ny,n_expected+1);

    %  ============================== 
    if s.mahalanobis
        pi_m = zeros(1,s.nLandmarks+1); 
        min_pi = 1000*mmd*ones(2,1);
    else % Euclidean
        pi_e = 10*s.min_radius;
    end

    % Compare current measurement to existing landmarks
    for k = 1:n_expected
        % Pull landmark from mean & Covariance position
        j = expected(1,k);

         % Predicted measurement to landmark
        [predZ(:,k),e(:,k),h] = rangeBearingModel(z(:,i), mu(1:s.nx), mu((s.ny*j+s.nx-1):(s.ny*j+s.nx)));
        % Projection Matrix for State & Landmark
        Fxj = [eye(s.nx),zeros(s.nx,s.ny*j-s.ny),zeros(s.nx,s.ny),zeros(s.nx,s.ny*(s.nLandmarks)-s.ny*j);
               zeros(s.ny,s.nx),zeros(s.ny,s.ny*j-s.ny),eye(s.ny),zeros(s.ny,s.ny*(s.nLandmarks)-s.ny*j)];
        % Projection Jacobian & Covariance update for current landmark
        predH(:,:,k) = h*Fxj;
        predPsi(:,:,k) = predH(:,:,k)*Sigma*predH(:,:,k)' + Qt;

        % ============================== 
        % Check likelihood of Measured Vs Expected Landmark 
        if s.mahalanobis                    

            pi_m(j) = e(:,k)'*(predPsi(:,:,k)\e(:,k));

            % Track best 2 associationa:
            if pi_m(j) < min_pi(1)
                min_pi(2) = min_pi(1);
                min_pi(1) = pi_m(j);
                l_idx = j;
                e_idx = k;
            elseif pi_m(j) < min_pi(2)
                min_pi(2) = pi_m(j);
            end             

        else % Euclidean
            ed = euclideanDistance(s,mu, z(:,i), j);
            if ed < pi_e
                pi_e(1) = ed;
                l_idx = j;
                e_idx = k;
            end
        end
    end
    % -------------------------------
    % Pull the best association
    if s.mahalanobis
        if (min_pi(2)/min_pi(1)) > best_asc
            % Save measurement number to associated landmark
            matches(2,i) = e_idx; % else marked zero, not associated
        end
    else % Euclidean
        if pi_e <= s.min_radius
            matches(2,i) = e_idx; % else marked zero, not associated
        end
    end

end
end