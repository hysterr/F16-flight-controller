%%AE4301P Assignment
%%Chapter8
%%Design of an automatic glideslope and flare controller

clear;
altitude=5000;
velocity=300;
x_a=0;
FindF16Dynamics;
close all;
A = A_longitude_lo([1,2,3,4,5],[1,2,3,4,5]);
B = A_longitude_lo([1,2,3,4,5],[6,7]);
C = C_longitude_lo([1,2,3,4,5],[1,2,3,4,5]);
D = C_longitude_lo([1,2,3,4,5],[6,7]);
%% Trimmed Values
ht_trim = trim_state_lin(3);
theta_trim = trim_state_lin(5)*180/pi;
velocity_trim = trim_state_lin(7);
%velocity_trim = 221;
alpha_trim = trim_state_lin(8)*180/pi;
q_trim = trim_state_lin(11)*180/pi;
%%% Glide Slope Control Parameters
runway_altitude= 3000;
time_interception = 10; %time before glideslope interception
glide_slope_ref = 3;
%%% Flare Control Parameters
flaretouchdowntime = 3; %times of time constant
x1= 700;
tau = x1/(flaretouchdowntime*velocity_trim-velocity_trim*cos(glide_slope_ref*pi/180));
x2=velocity_trim*cos(glide_slope_ref*pi/180)*tau;
hf=x2*tan(glide_slope_ref*pi/180)+10;
%%Run Simulation
sim glideslope_and_flare;
%%Plots
totaldistance = ((ht_trim-runway_altitude)/tan(3*pi/180))+time_interception*velocity_trim;
figure();
plot(s_hor_sim(:,2),altitude_sim(:,2));
hold on;
plot(s_hor_sim(:,2),(totaldistance-s_hor_sim(:,2))*tan(3*pi/180),'--r');
grid on;
grid minor;
title('Flight path and Glideslope reference');
xlabel('horizontal distance from starting point (ft)');
ylabel('altitude above runway (ft)');
legend('Fligt Path','Glideslope Reference');
hold off;
figure();
plot(v_vert_sim(:,1),v_vert_sim(:,2));
title('Vertical Velocity Time Response');
xlabel('time(s)');
ylabel('vertical velocity (ft/s)');
grid on;
grid minor;
figure();
plot(h_sim(:,1),h_sim(:,2));
title('Height Time Response');
xlabel('time(s)');
ylabel('height (ft)');
grid on;
grid minor;
figure();
plot(v_sim(:,1),v_sim(:,2));
title('Velocity Time Response');
xlabel('time(s)');
ylabel('velocity (ft/s)');
grid on;
grid minor;
figure();
plot(theta_sim(:,1),theta_sim(:,2));
title('Pitch Angle Time Response');
xlabel('time(s)');
ylabel('pitch angle (deg)');
grid on;
grid minor;
figure();
plot(alpha_sim(:,1),alpha_sim(:,2));
title('Angle of Attack Time Response');
xlabel('time(s)');
ylabel('angle of attack (deg)');
grid on;
grid minor;
figure();
plot(q_sim(:,1),q_sim(:,2));
title('Pitch Rate Time Response');
xlabel('time(s)');
ylabel('pitch rate (deg/s)');
grid on;
grid minor;
figure();
plot(glideslope_sim(:,1),glideslope_sim(:,2));
title('Glideslope Error Time Response');
xlabel('time(s)');
ylabel('glideslope error angle (deg)');
grid on;
grid minor;
%%END
