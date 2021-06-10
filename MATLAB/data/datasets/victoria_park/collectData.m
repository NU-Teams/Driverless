function events = collectData()
%% Description
% Collects and sorts Victoria Park dataset into ordered measurements, 
% outputs each event labeled as:
%   lsr: Raw Laser data [range] [maxRange 75m, 180*, 1/2 degree resolution]
%   gps: GPS data [m] [lon, lat]
%   dr: Dead reckoning [speed, steering]

%%
% Load data
load aa3_dr.mat
load aa3_gpsx.mat
load aa3_lsr2.mat

% Group data
lsr = [TLsr,LASER];
gps = [timeGps,Lo_m,La_m];
dr = [time,speed,steering];

% Find first & last time steps
t_min = min([lsr(1,1),gps(1,1),dr(1,1)]);
t_max = max([lsr(end,1),gps(end,1),dr(end,1)]);

% Initialise cell
events = {};

% Initialise index
lsr_idx = 1;
gps_idx = 1;
dr_idx = 1;
e_idx = 1;

% Sort events
for t = t_min:t_max
    if t == dr(dr_idx,1)
        events{e_idx,1} = 'dr';
        events{e_idx,2} = dr(dr_idx,:); 
        e_idx = e_idx + 1;
        
        if dr_idx < length(time)
            dr_idx = dr_idx + 1;
        end
    end  
    
    if t == lsr(lsr_idx,1)
        events{e_idx,1} = 'laser';
        events{e_idx,2} = lsr(lsr_idx,:);        
        e_idx = e_idx + 1;
        
        if lsr_idx < length(TLsr)
            lsr_idx = lsr_idx + 1;
        end 
    end
    if t == gps(gps_idx,1)
        events{e_idx,1} = 'gps';
        events{e_idx,2} = gps(gps_idx,:); 
        e_idx = e_idx + 1;
        
        if gps_idx < length(timeGps)
            gps_idx = gps_idx + 1;
        end
    end  
end
%% end of function
end