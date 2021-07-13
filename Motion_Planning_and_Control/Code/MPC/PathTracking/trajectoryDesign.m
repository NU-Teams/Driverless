function traj = trajectoryDesign(t, X)

% Ensure number of intercept times and waypoint pages match
assert(length(t) == size(X,3));

traj.t          = t;
traj.X          = X;

traj.nDim    	= size(X,1);         	% Dimension of splines
traj.nOrder   	= 2*size(X,2) - 1;      % Order of polynomial
traj.nWpts    	= length(t);            % Number of waypoints
traj.nSplines  	= traj.nWpts - 1;    	% Number of splines

% Derivative operator
traj.D = diag(1:traj.nOrder,-1);

% Each page T(:,:,i) = [t, D*t, D^2*t, ...]
traj.T = zeros(traj.nOrder+1,(traj.nOrder+1)/2,traj.nWpts);
for i = 1:traj.nWpts
    traj.T(:,1,i) = t(i).^(0:traj.nOrder).';
    for j = 1:(traj.nOrder-1)/2
        traj.T(:,j+1,i) = traj.D*traj.T(:,j,i);
    end
end

traj.C = zeros(traj.nDim,traj.nOrder+1,traj.nSplines);
for i = 1:traj.nSplines
    traj.C(:,:,i) = [traj.X(:,:,i) traj.X(:,:,i+1)]/[traj.T(:,:,i) traj.T(:,:,i+1)];
end
