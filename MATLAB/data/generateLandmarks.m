function s = generateLandmarks(s)
%% Description:
    % Creates landmarks based on an pre-exising track (passed through a structure)
%% Inputs:
    % s: Structure containing [list important structure features]
%% Outputs:
    % s: Structure updated w/ tracks Landmarks xy position representing
    %    track cones
%% Notes:
    % - 
%% References:
    %

%% Run run run
% load track data
try
    load(s.track,'xy'); % Labeled xy, check this with other datasets
catch
    fprintf('generateLandmarks: Error: Looking for label "xy"\n')
%     load(s.track,'matrix name'); % Labeled xy, check this with other datasets
end

% Pull params
if size(xy,1) < size(xy,2)% in row form
    x = xy(1,:);
    y = xy(2,:);
else % in column form
    x = xy(:,1);
    y = xy(:,2);
end

% Initialise
N = length(x);
L = []; % This will dynamically grow

% Parameters (maybe pass in s structure??)
coneDistance = 3; % [m] from centreline
variance = [0.05;0.01]; % [] variance in lengrh and angle (simulates a cone being placed "roughly" in the right spot)
gap = 10; % distance between sequential cones
dsum = 0;
dInnerSum = 0;
dOuterSum = 0;


d = 0;

%% Set first landmarks (knowledge that pose is 0 rads)
for i = 1:N
    if i == 1 % Initialise 1st set of landmarks at start point
       j = 0;
       % find change in direction (find starting direction)
       while d < gap/10
           dx = x(i+j) - x(i);
           dy = y(i+j) - y(i);
           d = sqrt(dx^2+dy^2);
           j = j + 1;
       end
       theta = atan2(dy, dx);
       
       % add ransom 'noise' to cone placement
       c = coneDistance + variance(1)*randn;
       a = theta + variance(2)*randn;
       
       L1 = [x(i) + c*sin(a); y(i) - c*cos(a)];
       L2 = [x(i) - c*sin(a); y(i) + c*cos(a)];
       L = [L,L1,L2];

       % Update intial point
       x0 = x(i);
       y0 = y(i);
       
       % Inner/outer constraints
       inner = [x(i) + coneDistance*sin(theta); y(i) - coneDistance*cos(theta)];
       outer = [x(i) - coneDistance*sin(theta); y(i) + coneDistance*cos(theta)];

    else
       if i < j
           dx = x(j+1) - x(1);
           dy = y(j+1) - y(1);
           d= 0;
       else
           dx = x(i) - x(i-1);
           dy = y(i) - y(i-1);
           d = sqrt(dx^2+dy^2);
       end
       theta = atan2(dy, dx);
       
       % Inner/outer constraints
       inner = [x(i) + coneDistance*sin(theta); y(i) - coneDistance*cos(theta)];
       outer = [x(i) - coneDistance*sin(theta); y(i) + coneDistance*cos(theta)];
   
       dxInner = inner(1) - inner_prev(1);
       dyInner = inner(2) - inner_prev(2);       
       dInner = sqrt(dxInner^2 + dyInner^2);
       
       dxOuter = outer(1) - outer_prev(1);
       dyOuter = outer(2) - outer_prev(2);       
       dOuter = sqrt(dxOuter^2 + dyOuter^2);
           
       % accounts for hairpin turns & corners
       dsum = dsum + d;
       dInnerSum = dInnerSum + dInner;
       dOuterSum = dOuterSum + dOuter;
       
     
       if dsum >= gap || dInnerSum >= 1.2*gap || dOuterSum >= 1.2*gap
           theta = atan2(dy, dx);
           % add ransom 'noise' to cone placement
           c = coneDistance + variance(1)*randn;
           a = theta + variance(2)*randn;
           
           % Landmark candidates
           L1 = [x(i) + c*sin(a); y(i) - c*cos(a)];
           L2 = [x(i) - c*sin(a); y(i) + c*cos(a)];
           
           % I want logic that checks for existing cones (exclude the last set)
           % This is incase gps data has overlap data, ie a 2nd lap
           % I also want to exclude the previous 2 sets of landmarks
           canPlace = true; 
           for k = 1:2:(size(L,2)-6)
               % Check candidates against previous markers
               dx = L1(1) - L(1,k);
               dy = L1(2) - L(2,k);
               dL1 = sqrt(dx^2+dy^2);
               
               dx = L2(1) - L(1,k+1);
               dy = L2(2) - L(2,k+1);
               dL2 = sqrt(dx^2+dy^2);
               
               if dL1 < gap || dL2 < gap
                   canPlace = false; 
               end
           end
           if canPlace
                L = [L,L1,L2];
           end

           % Update intial point
           x0 = x(i);
           y0 = y(i);
           dsum = 0;
           dInnerSum = 0;
           dOuterSum = 0;
       end
    end
    inner_prev = inner;
    outer_prev = outer;
end

% Plot the landmarks
plot(L(1,:),L(2,:),'k.')
hold on
plot(x,y,'b-')
xmin = min(L(1,:));
xmax = max(L(1,:));
ymin = min(L(2,:));
ymax = max(L(2,:));
xext = sqrt(abs(xmax-xmin)); xmin = xmin-xext; xmax = xmax+xext;
yext = sqrt(abs(ymax-ymin)); ymin = ymin-yext; ymax = ymax+yext;
axis([xmin,xmax,ymin,ymax])
axis equal
grid on
% squareMin = min([xmin,ymin]);
% squareMax = max([xmax,ymax]);
% axis([squareMin,squareMax,squareMin,squareMax]);
drawnow

s.gt.landmarks = L';


%% end of function
% end