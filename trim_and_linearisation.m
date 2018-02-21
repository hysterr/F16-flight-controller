%%AE4301P Assignment
%%Chapter 5
%Elevator Deflection to Normal Reaction Transfer Function and StepResponse
clear;
altitude = 15000;
velocity = 500;
%Response for accelerometer position x_a=0;
x_a=0;
FindF16Dynamics;
close all;
trf=tf(SS_long_lo);
sprintf('Transfer Function (Elevator Deflection to Normal Acceleration)');
e2an=trf(6,2);
disp('an to elevator deflection transfer function')
e2an=minreal(zpk(e2an));
sprintf('Poles');
poles=pole(e2an);
sprintf('Zeros');
zeros=zero(e2an);

%Step Response Plot

eldef = 10;
t=0:0.001:10;
clear func;
for i=1:1:10001
    if(i>=200)
        func(i) = -1*eldef*pi/180;
    else
        func(i)=0;
    end
end
[y,t]=lsim(e2an,func,t) ;
plot(t,y)
set(gcf, 'Position', get(0, 'Screensize'));
grid on;
grid minor;
title('Normal Acceleration Response (-10deg Elevator Deflection)');
xlabel('time(s)');
ylabel('Normal Acceleration an (g units)');
% print -depsc NormalAcc;

% for different x_a

x_a_v = [0 5 5.9 6 7 15];
color= ['k' 'r' 'g' 'b' 'm' 'c'];
%figure(1)
%hold on
for ii=1:1:6
    x_a=x_a_v(ii);
    FindF16Dynamics;
    %close all;
    trf=tf(SS_long_lo);
    
    e2an_v=trf(6,2);

    e2an=minreal(zpk(e2an_v)); 
    %poles and zeros being stored in matrix variable
    poles_v(ii,:)=(pole(e2an_v)).';
    zeros_v(ii,:)=(zero(e2an_v)).';
    
    %Step Response Plot
    eldef = 10;
    t=0:0.001:10;
    clear func;
    for i=1:1:10001
        if(i>=200)
            func(i) = -1*eldef*pi/180;
        else
            func(i)=0;
        end
    end
    [y,t]=lsim(e2an_v,func,t) ;
    plot(t,y,color(ii)) 
    hold on
    clear t;
    clear y;
end
set(gcf, 'Position', get(0, 'Screensize'));
grid on;
grid minor;
title('Normal Acceleration Response (-10deg Elevator Deflection)');
xlabel('time(s)');
ylabel('Normal Acceleration an (g units)');
legend('x_a=0ft','x_a=5ft','x_a=5.9ft','x_a=6ft','x_a=7ft','x_a=15ft');
hold off;
%print -depsc NormalAcc;
    
    
    
    
    


% Aaclo = A_longitude_lo([2,3,4,5],[2,3,4,5])
% Baclo = A_longitude_lo([2,3,4,5],[6,7])
% Caclo = C_longitude_lo([2,3,4,5],[2,3,4,5])
% Daclo = C_longitude_lo([2,3,4,5],[6,7])
% Aacla = A_lateral_lo([1,4,5,6],[1,4,5,6])
% Bacla = A_lateral_lo([1,4,5,6],[7,8,9])
% eigAaclo = eig(Aaclo) % 
% eigAacla = eig(Aacla)
% 
% %% State space for short period
% Asp= Aaclo([3,4],[3,4]);
% Bsp= Baclo([3,4],[1,2]);
% Bsp_del=Bsp([1,2],2);
% Csp= Caclo([3,4],[3,4]);
% Dsp= Daclo([3,4],[1,2]);
% Dsp_del=Dsp([1,2],2);
% sp_sys= ss(Asp,Bsp,Csp,Dsp);
% sp_sys_del= ss(Asp,Bsp_del,Csp,Dsp_del);
% four_sys= ss(Aaclo,Baclo,Caclo,Daclo);
% sp_tf= tf(sp_sys);
% sp_del_tf=tf(sp_sys_del);
% four_tf= tf(four_sys);
% [atf btf]=ss2tf(Asp,Bsp,Csp,Dsp,2);
% Ttheta2=atf(2,2)/atf(2,3);
% 
% t = 50;
% figure;
% step(sp_tf(1,2)*pi/180,t);
% hold on;
% step(four_tf(3,2)*pi/180,t);
% hold on;
% legend('2nd Order System','4th Order System')
% hold off;
% title('Angle of Attack Response');
% 
% 
% figure;
% step(sp_tf(2,2)*pi/180,t);
% hold on;
% step(four_tf(4,2)*pi/180,t);
% hold on;
% legend('2nd Order System','4th Order System')
% hold off;
% title('Pitch Rate response');
% %%Pole Placement
% pole1 = -0.5*5.4864 + 5.4864*sqrt(0.25-1)
% pole2 = -0.5*5.4864 - 5.4864*sqrt(0.25-1)
%  
% K = place(Asp, Bsp_del, [pole1,pole2]);
% %%New Closed Loop TF
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
% %%%% Lead Lag Filter etc
% 
% Tq_dele=sp_del_tf(2);
% 
% Ttheta2_req=1/(0.75*5.4864);
% % Lead Lag Filter
% ll_tf=tf([Ttheta2_req,1],[Ttheta2,1]);
% tf_overall=tf_cl*ll_tf;
% 
% 
% alphagust = 15/velocity;
% dele_max = K(1)*alphagust
% 
% %question 5: the time constant cannot be changed by pole placement and
% %therefore it has to be modified with a filter outside the loop.
% 
% 
% 
% % %% Phugoid
% % l1=eigAaclo(1);
% % l2=eigAaclo(2);
% % wn_phu=abs(l1);
% % T_phu=-real(l1)/wn_phu;
% % P_phu=2*pi/imag(l1);
% % Thald_phu=-log(2)/real(l1);
% % 
% % t = 1500;
% % TF = tf(wn_phu^2, [1, 2*T_phu*wn_phu, wn_phu^2]);
% % figure;
% % step(TF,t);
% % title('Phugoid');
% % grid on;
% % 
% % 
% % %% Short Period
% % l1=eigAaclo(3);
% % l2=eigAaclo(4);
% % wn_shp=abs(l1);
% % T_shp=-real(l1)/wn_shp;
% % P_shp=2*pi/imag(l1);
% % Thald_shp=-log(2)/real(l1);
% % 
% % t = 10;
% % TF = tf(wn_shp^2, [1, 2*T_shp*wn_shp, wn_shp^2]);
% % figure;
% % step(TF,t);
% % title('Short Period');
% % grid on;
% % 
% % %% Dutch Roll
% % l1=eigAacla(1);
% % l2=eigAacla(2);
% % wn_dr=abs(l1);
% % T_dr=-real(l1)/wn_dr;
% % P_dr=2*pi/imag(l1);
% % Thald_dr=-log(2)/real(l1);
% % 
% % t = 25;
% % TF = tf(wn_dr^2, [1, 2*T_dr*wn_dr, wn_dr^2]);
% % figure;
% % step(TF,t);
% % title('Dutch roll');
% % grid on;
% % 
% % %%Aperiodic Eigen Motions
% % 
% % %Spiral
% % l1=eigAacla(4);
% % tou_sp=-1/real(l1);
% % wn_sp=abs(l1);
% % Thalf_sp=-log(2)/real(l1);
% % 
% % t = 1500;
% % TF = tf(1, [tou_sp, 1]);
% % figure;
% % step(TF,t);
% % title('Spiral');
% % grid on;
% % 
% % %Aperiodic Roll
% % 
% % l1=eigAacla(3);
% % tou_roll=-1/real(l1);
% % wn_roll=abs(l1);
% % Thalf_roll=-log(2)/real(l1);
% % 
% % t = 10;
% % TF = tf(1, [tou_roll, 1]);
% % figure;
% % step(TF,t);
% % title('Aperiodic roll');
% % grid on;
% %%% CHapter 8
% 
