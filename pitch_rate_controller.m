%%AE4301P Assignment
%%Chapter 7
%Design of a Pitch Rate Controller
close all;
clear;
altitude = 20000;
velocity = 300;
x_a=0;
FindF16Dynamics;

%LTI model reduction

Aaclo = A_longitude_lo([2,3,4,5],[2,3,4,5]);
Baclo = A_longitude_lo([2,3,4,5],[6,7]);
Caclo = C_longitude_lo([2,3,4,5],[2,3,4,5]);
Daclo = C_longitude_lo([2,3,4,5],[6,7]);
 
%% State space for short period
Asp= Aaclo([3,4],[3,4]);
Bsp= Baclo([3,4],[1,2]);
Bsp_del=Bsp([1,2],2);
Csp= Caclo([3,4],[3,4]);
Dsp= Daclo([3,4],[1,2]);
Dsp_del=Dsp([1,2],2);
sp_sys= ss(Asp,Bsp,Csp,Dsp);
sp_sys_del= ss(Asp,Bsp_del,Csp,Dsp_del);
four_sys= ss(Aaclo,Baclo,Caclo,Daclo);
sp_tf= tf(sp_sys);
sp_del_tf=tf(sp_sys_del);
four_tf= tf(four_sys);
[atf btf]=ss2tf(Asp,Bsp,Csp,Dsp,2);
Ttheta2=atf(2,2)/atf(2,3);
% %Plot pitch rate and aoa response : 2nd order vs 4th order
% 
tt = 60;
opt = stepDataOptions('StepAmplitude',-1);
figure;
[y,t]=step(sp_tf(1,2),opt,tt);
plot(t,y);
hold on;
[y,t]=step(four_tf(3,2),opt,tt);
plot(t,y);
hold on;
xlabel('Time (s)');
ylabel('Angle of attack (deg)')
legend('2nd Order System','4th Order System')
hold off;
title('4th Order vs 2nd Order system: Angle of Attack Response');
% 
% 
figure;
[y,t]=step(sp_tf(2,2),opt,t);
plot(t,y);
hold on;
[y,t]=step(four_tf(4,2),opt,t);
plot(t,y);
hold on;
xlabel('Time (s)');
ylabel('Pitch rate (deg/s)')
legend('2nd Order System','4th Order System')
hold off;
title('4th Order vs 2nd Order system: Pitch Rate response');

%%Pole Placement
%Required poles
pole1 = -0.5*2.7432*2 + 2.7432*2*sqrt(0.25-1);
pole2 = -0.5*2.7432*2 - 2.7432*2*sqrt(0.25-1);
%Calculate Gains
K = place(Asp, Bsp_del, [pole1,pole2]);

%Validation against windgust 
alphagust = 15/velocity;
dele_max = K(1)*alphagust;


%%%% Lead Lag Filter etc

Tq_dele=sp_del_tf(2);

Ttheta2_req=1/(0.75*2.7432);
% Lead Lag Filter
ll_tf=tf([Ttheta2_req,1],[Ttheta2,1]);
%tf_overall=tf_cl*ll_tf;
% 
tf_resultant = -7.2145*ll_tf*tf([3.305s 1],[1 2.7432 7.5251]);
tf_resultant=minreal(zpk(tf_resultant));
tf_pitchangle=tf_resultant*tf([1],[1 0]);

tt = 20;
opt = stepDataOptions('StepAmplitude',-1);
figure;
[y,t]=step(tf_resultant,opt,tt);
plot(t,y);
xlabel('Time (s)');
ylabel('Pitch rate (deg/s)')
title('Pitch Rate time response');

tt = 20;
opt = stepDataOptions('StepAmplitude',-1);
figure;
[y,t]=step(tf_pitchangle,opt,tt);
plot(t,y);
xlabel('Time (s)');
ylabel('Pitch angle (deg)')
title('Pitch angle time response');

CAP_new=7.5251*0.4861/(300/32.18504);
CAP_old=1.0510*3.3047/(300/32.18504);

DBqss_new=0.4861-2*0.5/2.7432;
DBqss_old=3.3047-2*0.4028/1.0252;
% END

%%Ignore

%%New Closed Loop TF
% A_cl = Asp-Bsp_del*K;
% sys_cl=ss(A_cl,Bsp_del,Csp,Dsp_del);
% tf_cl=tf(sys_cl);
% 
% %%%%% Pole Placement through root locus
% Kalpha=K(1);
% Kq=K(2);
% Talpha=sp_del_tf(1);
% Tq=sp_del_tf(2);
% Tq_closed=Tq/((1+Kalpha*Talpha)*(1+Kalpha*Talpha+Tq*Kq));
% 
% 

