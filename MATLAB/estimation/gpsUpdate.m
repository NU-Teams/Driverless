function [mu, Sigma] = gpsUpdate(s, mu, Sigma, z)

sigma_gps = 2e2;


    Qt = [sigma_gps^2, 0;
           0, sigma_gps^2];
    s.nLandmarks = length(mu((s.nx+1):end))/2;
    [predZ,e,h] = simpleGpsModel(z, mu(1:s.nx));
    % Projection Matrix for State & Landmark
    j = s.nLandmarks;
%         Fxj = [eye(s.nx),zeros(s.nx,s.ny*j-s.ny),zeros(s.nx,s.ny),zeros(s.nx,s.ny*(s.nLandmarks+1)-s.ny*j);
%                zeros(s.ny,s.nx),zeros(s.ny,s.ny*j-s.ny),eye(s.ny),zeros(s.ny,s.ny*(s.nLandmarks+1)-s.ny*j)];
    Fxj = [eye(s.nx),zeros(s.nx,s.nLandmarks*2)];
    predH = h*Fxj;
    predPsi = predH*Sigma*predH' + Qt;
%         Hit = predH(:,1:s.nLandmarks*s.ny+s.nx,l_idx);
    Hit = predH;
    % Compute the kalman Gain
    Kit = Sigma*Hit'/predPsi;
    % Update mean/covariance
    mu = mu + Kit*e;
    Sigma = (eye(s.nx+s.ny*s.nLandmarks)-Kit*Hit)*Sigma;
   
%% end of function
end
