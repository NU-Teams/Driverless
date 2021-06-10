function z = getLandmarks(LASER);
%% LiDAR Feature Detect
AAr = [0:360]*pi/360; % 180 Degree field of view at 1/2 degree 
lsr = uint16(LASER);
CAAA = cos(AAr);
SAAA = sin(AAr);

nc = 9 ; 
aaa = [0:nc]*2*pi/nc ; 
pCircles = [ cos(aaa); sin(aaa) ] ;  % xy coordinates of a circle

% Masks for filtering
Mask13 = uint16(2^13 -1);
MaskA  = bitcmp(Mask13,'uint16');
% bitwise mask
try
    RR = double(  bitand( Mask13,lsr) ) ;
    a  = uint16(  bitand( MaskA ,lsr) ) ;
    ii = find(a>0) ;
    % Convert range [m]
    RR = RR/100 ;
    z=detectTreesI16(RR) ; %Landmark: range, angle & est size
    z = z(1:2,:);
catch
    z = [];
end

%% end of function
end