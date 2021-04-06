function u  = input_robotModel2Wheels_loop1(t,u, res)
u = zeros(size(u));

if t < 100*res
    u(1) = 15;
    u(2) = 15;
elseif t < 150*res
    u(1) = 7;
    u(2) = 7;
elseif t < 200*res
    u(1) = 7;
    u(2) = 5.5;
elseif t < 250*res
    u(1) = 5.5;
    u(2) = 8;
elseif t < 300*res
    u(1) = 7;
    u(2) = 7;
elseif t < 350*res
    u(1) = 0;
    u(2) = 5;
elseif t < 425*res
    u(1) = 7;
    u(2) = 4;
elseif t < 500*res
    u(1) = 7;
    u(2) = 10;
elseif t < 525*res
    u(1) = 3;
    u(2) = 15;
elseif t < 600*res
    u(1) = 10;
    u(2) = 3;
elseif t < 625*res
    u(1) = 3;
    u(2) = 10;
elseif t < 650*res
    u(1) = 0;
    u(2) = 0;
elseif t < 680*res
    u(1) = 8;
    u(2) = 10;
elseif t < 710*res
    u(1) = 8;
    u(2) = 4;
elseif t < 750*res
    u(1) = 6;
    u(2) = 7;
elseif t < 775*res
    u(1) = 6;
    u(2) = 6;
elseif t < 800*res
    u(1) = 5;
    u(2) = 6;
elseif t < 825*res
    u(1) = 10;
    u(2) = 12;
elseif t < 850*res
    u(1) = 8;
    u(2) = 8;
elseif t < 860*res
    u(1) = 12;
    u(2) = 4;
elseif t < 880*res
    u(1) = 0;
    u(2) = 0;
elseif t < 900*res
    u(1) = 5;
    u(2) = 0;
elseif t < 940*res
    u(1) = 4;
    u(2) = 5.5;
elseif t < 970
    u(1) = 4; % u(1,t) + -5;
    u(2) = 4.25; % u(2,t) + 5; 
elseif t<980
    u(1) = 4; % u(1,t) + -5;
    u(2) = 5; % u(2,t) + 5; 
else
    u(1) = 5; % u(1,t) + -5;
    u(2) = 5; % u(2,t) + 5; 
end
end