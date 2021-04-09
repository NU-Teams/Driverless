function [mu,Sigma] = landmarkManagement(mu,Sigma,slm, s, type)
%% Description:
    % EKF SLAM Algorithm, predicts and filters state means & covariances.
%% Inputs:
    % mu: 
    % Sigma
    % slm: seen landmark (slm)
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

% Read binary file
try
    file = fopen(s.fileName, 'r');
    lmmv = fread(file,'int8');
    fclose(file);
catch
    lmmv = [];
end

ul = 50;
ll = -10;
switch type
    case 'delete'
        % check lmmv, if over limit, delete landmark
        
        % Check lmmv size (if new lm's are seen)
        adj = (length(slm) - length(lmmv));
        if adj > 0
            lmmv = [lmmv; zeros(adj,1)];
        end
        % Update lmmv
        lmmv = lmmv + 3.*slm;
        lmmv(lmmv(:)>ul) = ul;
        % Delete Landmark
        mask = true;
        while mask
            % Number of landmarks
            M = length(mu(s.nx+1:end))/2;
            for o = 1:M % can vectorise this
                if lmmv(o) < ll
                    % reduce landmark size for recheck
                    %M = M - 1; % double check this orr resize in for loop
                    disp('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> landmark deleted')
                    % resize vectors/matrix (deleting o)
                    slm = [...
                        slm(1:(o-1));...
                        slm(o+1:end)];
                    lmmv = [...
                        lmmv(1:(o-1));...
                        lmmv(o+1:end)];
                    
                    mu = [...
                        mu(1:s.nx+s.ny*(o-1));...
                        mu(s.nx+s.ny*o+1:end)];
                    Sigma = [...
                        Sigma(1:s.nx+s.ny*(o-1),1:s.nx+s.ny*(o-1)),...
                        Sigma(1:s.nx+s.ny*(o-1),s.nx+s.ny*o+1:end);...
                        Sigma(s.nx+s.ny*o+1:end,1:s.nx+s.ny*(o-1)),...
                        Sigma(s.nx+s.ny*o+1:end,s.nx+s.ny*o+1:end)];
                    
                    break;
                % Else no more landmarks to be deleted 
                elseif o == M
                    mask = false; 
                end
            end
        end
        % WRITE TO BINARY FILE
        file = fopen(s.fileName,'w');
        fwrite(file, lmmv,'int8');
        fclose(file);
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
        
        % Number of landmarks
        M = length(mu(s.nx+1:end))/2;            
        
        for o = 1:M % can I vectorise this
            % Convert landmark mean to range bearing
            dx = mu(s.nx+s.ny*o-1) - mu(1);
            dy = mu(s.nx+s.ny*o) - mu(2);
            q = dx^2 + dy^2;
            % Psuedo range/bearing model for landmark
            ztemp = [sqrt(q);
                     wrapTo2Pi(atan2(dx,dy) - mu(3))];
            
            % Is landmark expected to be seen?
            if (ztemp(1) <= s.par_rb.maxRange) && (ztemp(2) >= (360-s.par_rb.angle/2)*pi/180 || ztemp(2) <= (s.par_rb.angle/2)*pi/180)
                % Add a counter
                lmmv(o) = lmmv(o) - 2;
            end
        end
        % write the binary file
        file = fopen(s.fileName,'w');
        fwrite(file, lmmv,'int8');
        fclose(file);
    otherwise
        % uh oh, shouldn't get here
end
%% end of function
end