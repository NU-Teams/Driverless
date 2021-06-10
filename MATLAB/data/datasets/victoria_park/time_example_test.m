close all
clear all
clc

load aa3_gpsx
load aa3_lsr2
load aa3_dr.mat

dmin = 0.2; 
TLsr = double(TLsr);

figure(1)
plot(TLsr,'r.')
hold on
plot(timeGps,'b.')
plot(time,'g.')

if length(time) > length(timeGps)
    sz = length(time);
else
    sz = length(timeGps);
end
dl = 0;
tu = [NaN];

while timeGps(1) < time(1) - dl
    tu = [tu; NaN];
    dl = dl + 25;
end

sp = [tu;speed];
st = [tu;steering];
tu = [tu;time];

counter = 0;
g0 = TLsr(1);
tlsr = [g0];
lid = NaN(61933,361);
lid(1,:) = LASER(1,:);

for i = 2:length(TLsr)
    
    try
        while time(i+counter-1) < TLsr(i) || isnan(time(i+counter-1))
            tlsr = [tlsr; NaN];
%             lid(i+counter) = [lid; NaN(1,length(LASER(1,:)))];
            counter = counter + 1;
        end
        tlsr = [tlsr; TLsr(i)];
        lid(i+counter,:) = LASER(i,:);
    catch
%         tgps = [tgps; NaN];
%         tlsr = [tlsr; NaN];
    end
        
end


counter = 0;
g0 = timeGps(1);
tgps = [g0];
gps = [Lo_m(1),La_m(1)];

for i = 2:length(timeGps)
    
    try
        while time(i+counter-1) < timeGps(i) || isnan(time(i+counter-1))
            tgps = [tgps; NaN];
            gps = [gps;NaN(1,2)];

            counter = counter + 1;
        end
        tgps = [tgps; timeGps(i)];
        gps = [gps;Lo_m(i),La_m(i)];
    catch
%         tgps = [tgps; NaN];
%         tlsr = [tlsr; NaN];
    end
        
end

dl = 0;
tlsr = [NaN;tlsr];
lid = [NaN(1,length(LASER(1,:)));lid];
while timeGps(1) < TLsr(1) - dl
    tlsr = [NaN;tlsr];
    lid = [NaN(1,length(LASER(1,:)));lid];
    dl = dl + 25;
end

lid = [lid;NaN(length(tu)-length(tlsr),length(LASER(1,:)))];
tlsr = [tlsr;NaN(length(tu)-length(tlsr),1)];

gps = [gps;NaN(length(tu)-length(tgps),2)];
tgps = [tgps;NaN(length(tu)-length(tgps),1)];

figure(2)
plot(tlsr,'r.','MarkerSize',24)
hold on
plot(tgps,'b.','MarkerSize',18)
plot(tu,'g.','MarkerSize',8)

save('vp_input.mat','sp','st');
save('vp_lid.mat','lid');
save('vp_gps.mat','gps');