function [Deform] = TL2Deform_seg(D_TL,R_wrist,T_angle)
% Transmit  D_TL ([d_TL1, d_TL1, d_TL1]) to Deform([Bend_XY, D_L])   

% R_wrist=18;
% D_TL=[18*pi 9*pi 9*pi]
% T_angle=[0 pi*2/3 pi*4/3]

Bend_v=[D_TL'.*[cos(T_angle'),sin(T_angle')]]/(1.5*R_wrist);
Bend_XY=sum(Bend_v);
D_L=sum(D_TL)/3;
Deform=[Bend_XY,D_L];
end