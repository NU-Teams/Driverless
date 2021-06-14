function TCPCallback(TCPhandle,~)
%PARSEPACKET Summary of this function goes here
%   Read packet on a seprarate thread to prevent buffer overflow

% note that:
% 8 bits= 1 byte
% uint8 uses 8 bits or 1 bytes
% uint16 uses 16 bits or 2 bytes
% uint32 uses 32 bits or 4 bytes
% bytes/uint8 will later be recombined back to uin32
% 4 uint8 can be combined into 1 uint32

%   total bytes per packet 4*4+1*4+50*(2*2+4*3*8+1*3*8+1*8)+4*2+2*2 = 6632 bytes

    TCPhandle.UserData = read(TCPhandle,6632*220,'uint8');
end

