function [z,u,i] = getVictoriaPark()
%% Description:
    % Returns selected Process Model
%% Inputs:
    % opt: structure containing vehicleModel as string
    %      example: opt.vehicleModel = 'robotModel2Wheels'
    % x:   States of vehicle model (optional)
    % u:   Inputs of vehicle model (optional)
%% Outputs:
    % model: returns selected Model
%% Notes:
    % - Switch Case argurment, add to argument when adding new vehicle
    %   modesls
    %   Example:
    %   case 'modelNameHere'
    %       model = modelNameHere()
    % IMPORTANT: Maybe I will return opt with a symbollic model added to
    % the structure? I dunno. Figure out which is faster
%% References:
    % 
    

load('vp_input.mat');
load('vp_lid.mat');
load('vp_gps.mat');

z.lid = lid;
z.gps = gps;
u = [sp,st];
i.isGps = true;
i.isLidar = true;
i.Delta = 0.025;
%% end of function

% %% LiDAR Feature Detect
% AAr = [0:360]*pi/360; % 180 Degree field of view at 1/2 degree 
% L = size(LASER);
% Time = double(TLsr);
% 
% CAAA = cos(AAr);
% SAAA = sin(AAr);
% 
% nc = 9 ; 
% aaa = [0:nc]*2*pi/nc ; 
% pCircles = [ cos(aaa); sin(aaa) ] ;  % xy coordinates of a circle
% 
% % Masks for filtering
% Mask13 = uint16(2^13 -1);
% MaskA  = bitcmp(Mask13,'uint16');
% % bitwise mask
% RR = double(  bitand( Mask13,LASER(t,:)) ) ;
% a  = uint16(  bitand( MaskA ,LASER(t,:)) ) ;
% ii = find(a>0) ;
% % Convert range [m]
% RR = RR/100 ;
% xra=detectTreesI16(RR) ; %Landmark: range, angle & est size