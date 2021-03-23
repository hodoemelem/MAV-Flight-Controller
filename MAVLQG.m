%% for MAVMODEL8S: roll,pitch,yaw and Z states only
%%%%%%%%%%%%%%%%%%%%%%% X config%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;


%F=k*n^2
%M=b*n^2
%n is rps
g = 9.81; %m/s2
M = 1.280; %Kg
Ixx = 8.4471e-3; % kgm2
Iyy = 8.4471e-3; % kgm2
Izz = 9.988e-3; % kgm2
b = 2.6788e-6;  %drag constant
k = 1.8409e-4;  %lift constant
Ts = 10e-3;%sec

l = 0.1167; % link lenght in meter
%cM = [k k k k;l*k -l*k -l*k l*k ;-l*k -l*k l*k l*k;-b b -b b];
cM = [k k k k;l*k -l*k -l*k l*k ;-l*k -l*k l*k l*k;b -b b -b];
cMi = pinv(cM);
%hoverThrust = [M*g/4 M*g/4  M*g/4  M*g/4];
%hoverThrust = [0 0  0  0];
A = zeros(8);
B = zeros(8,4);

A(1,2) = 1;
A(3,4) = 1;
A(5,6) = 1;
A(7,8) = 1;


B(2,2) = 1/Ixx;
B(4,3) = 1/Iyy;
B(6,4) = 1/Izz;
B(8,1) = 1/M;

Q = eye(8)*1;
%measured sates
Q(1,1)=0.0044;
Q(3,3)= 0.0044;
Q(5,5)= 0.0044;
Q(7,7)= 0.25;
%vel.states vel.
Q(2,2)= 0.0010;
Q(4,4)= 0.0010;
Q(6,6)= 0.0010;
Q(8,8)= 0.0625;


R = zeros(4);
R(1,1) = 0.005102;
R(2,2) = 45;
R(3,3) = 45;
R(4,4) =47;

% % %measured sates
% Q(1,1)=0.00005;
% Q(3,3)= 0.00005;
% Q(5,5)= 0.00005;
% Q(7,7)= 0.00005;
% %UNMeASuRED states vel.
% Q(2,2)= 0;
% Q(4,4)= 0;
% Q(6,6)= 0;
% Q(8,8)= 0;
% 
% 
% R = zeros(4);
% R(1,1) = 0.01;
% R(2,2) = 1;
% R(3,3) = 1;
% R(4,4) = 1;

[K,S,e] = lqr(A,B,Q,R);
[Kd,Sd,ed] = lqrd(A,B,Q,R,Ts);

rollf = 0;
pitchf = 0;
yawf =0;
zf = 0.5;

roll = 0;
pitch = 0;
yaw = 0;
z = 0.2;
%x = [roll 0 pitch  0 yaw  0 z 0];
x = [0 0 0  0 0  0 0 0];

%% for MAVMODEL: full states with lqr
% clc;
% clear;
% g = 9.81; %m/s2
% M = 0.425; %Kg
% Ixx = 0.00104; % kgm2
% Iyy = 0.00104; % kgm2
% Izz = 0.0135; % kgm2
% A = zeros(12);
% B = zeros(12,4);
% 
% A(1,2) = 1;
% A(3,4) = 1;
% A(5,6) = 1;
% A(7,8) = 1;
% A(9,10) = 1;
% A(11,12) = 1;
% A(8,3) = g;
% A(10,1) = -g;
% 
% B(2,2) = 1/Ixx;
% B(4,3) = 1/Iyy;
% B(6,4) = 1/Izz;
% B(12,1) = 1/M;
% 
% Q = eye(12)*1;
% %Q(1,1)= 1;
% %Q(3,3)= 10;
% %UNMASEARED states vel.
% Q(2,2)= 0;
% Q(4,4)= 0;
% Q(6,6)= 0;
% Q(8,8)= 0;
% Q(10,10)= 0;
% Q(12,12)= 0;
% 
% R = zeros(4);
% R(1,1) = 1;
% R(2,2) = 1;
% R(3,3) = 1;
% R(4,4) = 0.1;
% 
% [K] = lqr(A,B,Q,R);
% 
% roll = 0;
% x = [roll 0 0 0 0 0 0 0 0 0 0 0];
% 
% disp(K);

%% for MAVMODEL8S: roll,pitch,yaw and Z states only
% %%%%%%%%%%%%%%%%%% + config%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc;
% clear;
% 
% 
% 
% g = 9.81; %m/s2
% M = 0.425; %Kg
% Ixx = 0.00104; % kgm2
% Iyy = 0.00104; % kgm2
% Izz = 0.0135; % kgm2
% b = 6.76208e-8;  %drag constant
% k = (1.8157e-4)/(4*pi*pi);  %lift constant
% 
% 
% 
% l = 0.21; % link lenght in meter
% cM = [k k k k;l*k 0 -l*k 0;0 -l*k 0 l*k;b -b b -b];
% 
% 
% cMi = pinv(cM);
% hoverThrust = [M*g/4 M*g/4  M*g/4  M*g/4];
% A = zeros(8);
% B = zeros(8,4);
% 
% A(1,2) = 1;
% A(3,4) = 1;
% A(5,6) = 1;
% A(7,8) = 1;
% 
% 
% B(2,2) = 1/Ixx;
% B(4,3) = 1/Iyy;
% B(6,4) = 1/Izz;
% B(8,1) = 1/M;
% 
% Q = eye(8)*1;
% %measured sates
% Q(1,1)=0.00005;
% Q(3,3)= 0.00005;
% Q(5,5)= 0.00005;
% Q(7,7)= 0.00005;
% %UNMeASuRED states vel.
% Q(2,2)= 0;
% Q(4,4)= 0;
% Q(6,6)= 0;
% Q(8,8)= 0;
% 
% 
% R = zeros(4);
% R(1,1) = 1;
% R(2,2) = 1;
% R(3,3) = 1;
% R(4,4) = 1;
% 
% [K] = lqr(A,B,Q,R);
% 
% 
% rollf = 0*(pi)/180.0;
% pitchf = 15*(pi)/180.0;
% yawf = 0*(pi)/180.0;
% zf = 1;
% 
% roll = 0*(pi)/180.0;
% pitch = 0*(pi)/180.0;
% yaw = 0*(pi)/180.0;
% z = 1;
% x = [roll 0 pitch  0 yaw  0 z 0];




%% for MAVMODEL4S: roll,pitch states only
%%%%%%%%%%%%%%%%%%%%%%% X config%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc;
% clear;
% 
% 
% %F=k*n^2
% %M=b*n^2
% %n is rps
% g = 9.81; %m/s2
% M = 1.295; %Kg
% Ixx = 6.5874e-3; % kgm2
% Iyy =  5.8822e-3; % kgm2
% Izz = 6.605e-3; % kgm2
% b = 2.6788e-6;  %drag constant
% k = 1.8409e-4;  %lift constant
% Ts = 6e-3;%sec
% l = 0.1167; % link lenght in meter
% cM = [k k k k;l*k -l*k -l*k l*k ;-l*k -l*k l*k l*k;-b b -b b];
% 
% cMi = pinv(cM);
% hoverThrust = [M*g/4 M*g/4  M*g/4  M*g/4];
% A = zeros(4,4);
% B = zeros(4,4);
% 
% A(1,2) = 1;
% A(3,4) = 1;
% 
% 
% 
% B(2,2) = 1/Ixx;
% B(4,3) = 1/Iyy;
% 
% 
% Q = eye(4)*1;
% % %measured sates
% Q(1,1)=0.0044;
% Q(3,3)= 0.0044;
% 
% 
% % Q(2,2)= 0;
% % Q(4,4)= 0;
% Q(2,2)= 0.0010;
% Q(4,4)= 0.0010;
% 
% 
% R = zeros(4);
% % R(1,1) = 0.005102;
% R(2,2) =  45;
% R(3,3) =  45;
% R(4,4) =  47;
% 
% % %measured sates
% % Q(1,1)=10;
% % Q(3,3)= 10;
% % 
% % 
% % Q(2,2)= 0;
% % Q(4,4)=0;
% % 
% % 
% % 
% % R = zeros(4);
% % R(1,1) = 20000;
% % R(2,2) =  20000;
% % R(3,3) =  20000;
% % R(4,4) =  20000;
% 
% [K,S,e] = lqr(A,B,Q,R);
% 
% [Kd,Sd,ed] = lqrd(A,B,Q,R,Ts);
% 
% rolld = 0;
% pitchd = 0;
% yawd =0;
% zd = 1;
% 
% roll = 0;
% pitch = 15;
% yaw = 0;
% z = 1;
% x = [roll 0 pitch  0 ];



%% for MAVMODELrollY: roll,y states only
% % NB: When you roll (about x-axis), you translate in y diretion.
% clc;
% clear;
% g = 9.81; %m/s2
% M = 0.425; %Kg
% Ixx = 0.00104; % kgm2
% Iyy = 0.00104; % kgm2
% Izz = 0.0135; % kgm2
% A = zeros(4);
% B = zeros(4,1);
% C = eye(4);
% D = eye(4,1);
% 
% A(1,2) = 1;
% A(3,4) = 1;
% A(4,1) = -g;
% 
% 
% B(2) = 1/Ixx;
% 
% Q = eye(4)*1;
% %measured sates
% Q(1,1)= 0.1;
% Q(3,3)= 0.1;
% 
% %UNMeASuRED states vel.
% Q(2,2)= 0;
% Q(4,4)= 0;
% 
% 
% 
% R = 10;
% 
% 
% [K] = lqr(A,B,Q,R);
% 
% rollf = 0*(pi)/180.0;
% yf = 1;%meter
% 
% roll = 0*(pi)/180.0;
% y = 0;
% x = [roll 0 y 0];

% clc;
% T = [0.226 0.287 0.347 0.420 0.495]*9.81;
% rps = [6630/60 7410/60 8220/60 8940/60 9660/60];
% %scatter(rps,T);
% p = polyfit(rps,T,2);
% y = polyval(p,rps);
% figure
% plot(rps,T,'o');
% hold on
% plot(rps,y);
% hold off









