%%AE4301P Assignment
%Chapter 6
%Open Loop Analysis

%LTI model reduction into 4th order form
close all;
clear;
altitude = 20000;
velocity = 300;
x_a=0;
FindF16Dynamics;

Aaclo = A_longitude_lo([2,3,4,5],[2,3,4,5]);
Baclo = A_longitude_lo([2,3,4,5],[6,7]);
Caclo = C_longitude_lo([2,3,4,5],[2,3,4,5]);
Daclo = C_longitude_lo([2,3,4,5],[6,7]);
Aacla = A_lateral_lo([1,4,5,6],[1,4,5,6]);
Bacla = A_lateral_lo([1,4,5,6],[7,8,9]);
Cacla = C_lateral_lo([1,4,5,6],[1,4,5,6]);
Dacla = C_lateral_lo([1,4,5,6],[7,8,9]);
eigAaclo = eig(Aaclo); % 
eigAacla = eig(Aacla);

%%Eigen Motion Identification
% Phugoid
l1=eigAaclo(1);
l2=eigAaclo(2);
wn_phu=abs(l1);
T_phu=-real(l1)/wn_phu;
P_phu=2*pi/imag(l1);
Thald_phu=-log(2)/real(l1);

t = 700;
TF = tf(wn_phu^2, [1, 2*T_phu*wn_phu, wn_phu^2]);
figure;
step(TF,t);
title('Phugoid');
grid on;
grid minor;


% Short Period
l1=eigAaclo(3);
l2=eigAaclo(4);
wn_shp=abs(l1);
T_shp=-real(l1)/wn_shp;
P_shp=2*pi/imag(l1);
Thald_shp=-log(0.5)/real(l1);

t = 20;
TF = tf(wn_shp^2, [1, 2*T_shp*wn_shp, wn_shp^2]);
figure;
step(TF,t);
title('Short Period');
grid on;
grid minor;

%% Dutch Roll
l1=eigAacla(1);
l2=eigAacla(2);
wn_dr=abs(l1);
T_dr=-real(l1)/wn_dr;
P_dr=2*pi/imag(l1);
Thald_dr=log(0.5)/real(l2);

t = 25;
TF = tf(wn_dr^2, [1, 2*T_dr*wn_dr, wn_dr^2]);
figure;
step(TF,t);
title('Dutch roll');
grid on;
grid minor;

%%Aperiodic Eigen Motions

%Spiral
l1=eigAacla(4);
tou_sp=-1/real(l1);
wn_sp=abs(l1);
Thalf_sp=log(0.5)/real(l1);

t = 500;
TF = tf(1, [tou_sp, 1]);
figure;
step(TF,t);
title('Spiral');
grid on;
grid minor;

%Aperiodic Roll

l1=eigAacla(3);
tou_roll=-1/real(l1);
wn_roll=abs(l1);
Thalf_roll=log(0.5)/real(l1);

t = 10;
TF = tf(1, [tou_roll, 1]);
figure;
step(TF,t);
title('Aperiodic roll');
grid on;
grid minor;
