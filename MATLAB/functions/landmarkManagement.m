function [mu,Sigma, lmmv] = landmarkManagement(mu,Sigma,lmmv, s, type)
%% Description:
    % EKF SLAM Algorithm, predicts and filters state means & covariances.
%% Inputs:
    % mu: 
    % Sigma
    % lmmv: landmark management vector (lmmv) (maybe include )
%% Outputs:
    % s: outputs updated filtered/predicted state mean/covariances w/ 
    %    added landmarks to state mean/covariances
%% Notes:
    % - 
%% References:
    % EKF SLAM algorithm with ML corresopences Table 10.2 pg 322,
    % Probabilistic Robotics, 2006, [Thrun et al]
    %

%% Lets try to manage...

% Psuedo code
% switch case, add / remove
% each iteration, we add a point to each landmark
% each iteration, we remove a point if we expected to see a landmark but
% failed.
% If there is no measurement to an expected landmark, then after so much
% evidence, we delete the landmark.
% perhaps a weighted system + 2 points and - 3 points?
% for a test, I will use 2 identical maps. but on the second maps, I will
% add 4 random (or intentionally placed) landmarks, at some point I will 
% switch to the original 

% https://fr.mathworks.com/help/matlab/ref/fread.html
% using binary files to read write landmark management vector. 
% Much faster, and this means I don't need to pass this vector through to
% the main_function, as it is recursive. 
% OR... I could add this to the state vector, but it will add unnecessary
% processing to my solution..

% int8 to save time/space (-128 to 127)
% Need to decide on upper and lower limits
ul = 120;
switch type
    case 'delete'
        % check lmmv, if over limit, delete landmark
    case 'seen'
        % landmark seen
        % to save read/write for EACH landmark, I should create a vector of
        % the 'seen' landmarks for this iteration.. then I'll add them
        % here..
        % so only 2 read/writes per iteration. 
        % so >> lmmv = lmmv + seen..
        % I alsoe need a check (incase a new landmark has been added),
        % basically they will be zeroed..
        % I can probably combine 'seen' & delete in the same step. 
        % 
        
    case 'expected'
        % token add. Assess each landmark with sensors field of view.
        % Read/Store binary file into vector
        
        M = length(mu(s.nx+1:end))/2;
        for o = 1:M % can vectorise this
            % Convert landmark mean to range bearing
            dx = mu(s.nx+s.ny*o-1) - mu(1);
            dy = mu(nx+ny*o) - mu(2);
            q = dx^2 + dy^2;
            % Psuedo range/bearing model for landmark
            ztemp = [sqrt(q);
                     wrapTo2Pi(atan2(dx,dy) - mu(3))];
            
            % Is landmark expected to be seen?
            if (ztemp(1) <= maxRange) && (ztemp(2) >= (360-maxbearing/2)*pi/180 || ztemp(2) <= (maxbearing/2)*pi/180)
                % Add a counter
                lmmv(o) = lmmv(o) + 2;
                % Sets upper limit (prevents going inf aka too large)
                if lmmv(o) > ul
                    lmmv(o) = ul;
                end
            end
        end
        % write the binary file
        
    otherwise
        % uh oh, shouldn't get here
end
        
%% end of function
end