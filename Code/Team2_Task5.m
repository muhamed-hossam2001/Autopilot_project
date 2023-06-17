%% Team2 Task4:
%% Start:
clc
clear vars
close all
%%  Excel Sheets Data:
tic
filename_density_L = 'Jetstar_FC2';     %% put here the location of your excel sheet
%filename_density_L = 'Boeing747_FC5';   %% put here the location of your excel sheet
aircraft_data=xlsread(filename_density_L,'B2:B61');     %% here B2:B61 means read the excel sheet from cell B2 to cell B61
%% Data needed for calculations:
%% Time Vector Parameters:
dt = aircraft_data(1);    
tfinal = aircraft_data(2); 
n =tfinal/dt+1;  % Number of iterations
t = (0:dt:tfinal)';
%% Initial Conditions:
Q = 78.4;
S = 542.5;
CD = 0.07;
thrust = Q*S*CD;
engine = 3700;
no_of_engines = 4;
full_thrust = no_of_engines*engine;
s0 = aircraft_data(4:15);
Vto = sqrt(s0(1)^2 + s0(2)^2 + s0(3)^2);    % Velocity total
aBV(:,1) = [aircraft_data(18);aircraft_data(20);Vto];
y(:,1) = [s0;aBV];  sdot0 = zeros(12,1);  k1 = sdot0 ;
u0   = y(1,1);  v0 = y(2,1);     w0 = y(3,1);       p0 = y(4,1);    q0 = y(5,1);       
r0 = y(6,1);    phi0 = y(7,1);   theta0 = y(8,1);   bsi0 = y(9,1);  B0   = y(14,1);
K = struct('u',s0(1),'v',s0(2),'w',s0(3),'p',s0(4),'q',s0(5),'r',s0(6),'phi',s0(7),'theta',s0(8),'psi',s0(9)...
,'X',s0(10),'Y',s0(11),'Z',s0(12),'alpha',s0(8),'beta',0,'total_speed',Vto,'Wd0t',0);
%% Control Actions Values:
da = aircraft_data(57);   % delta aileron
dr = aircraft_data(58);   % delta rudder
de = aircraft_data(59);   % delta elevator
dth = aircraft_data(60);  % delta Thrust
dc = [ aircraft_data(57:59) * pi/180 ; aircraft_data(60)];   % control inputs as a vector
%% Gravity, Mass % Inertia:
Thrust = 3700;
NO_TH = 4;
m = aircraft_data(51);    % mass
g = aircraft_data(52);    % gravity acceleration
Ixx = aircraft_data(53);  % moment of inertia about x
Iyy = aircraft_data(54);  % moment of inertia about y
Izz = aircraft_data(55);  % moment of inertia about z
Ixz = aircraft_data(56);  % product of inertia in xz plane
Ixy=0;                    % product of inertia in xy plane
Iyz=0;                    % product of inertia in yz plane
I = [Ixx , -Ixy , -Ixz ;...
    -Ixy , Iyy  , -Iyz ;...
    -Ixz , -Iyz , Izz  ];
%% Stability Derivatives for Longitudinal Motion:
SD_Long = aircraft_data(21:36);
Xu   = SD_Long(1);      Zu   = SD_Long(2);    Mu = SD_Long(3);
Xw   = SD_Long(4);      Zw   = SD_Long(5);    Mw = SD_Long(6);
ZwD  = SD_Long(7);      Zq   = SD_Long(8);
MwD  = SD_Long(9);      Mq   = SD_Long(10);
XDe  = SD_Long(11);     ZDe  = SD_Long(12);   MDe  = SD_Long(13);
XDth = SD_Long(14);     ZDth = SD_Long(15);   MDth = SD_Long(16);
%% Stability Derivatives for Lateral Motion:
SD_Lat_dash = aircraft_data(37:50);
Yv   = SD_Lat_dash(1);        YB   = SD_Lat_dash(2);    Yp  = 0;     Yr  = 0;
LB_  = SD_Lat_dash(3);        Lp_  = SD_Lat_dash(5);    Lr_ = SD_Lat_dash(7);
NB_  = SD_Lat_dash(4);        Np_  = SD_Lat_dash(6);    Nr_ = SD_Lat_dash(8);
YDaS = SD_Lat_dash(9);        YDrS = SD_Lat_dash(10);
LDa_ = SD_Lat_dash(11);       LDr_ = SD_Lat_dash(12);
NDa_ = SD_Lat_dash(13);       NDr_ = SD_Lat_dash(14);
G = 1/(1-(Ixz^2)/(Ixx*Izz));
LNDTLN = G*[1     ,Ixz/Ixx,0     ,0     ,0     ,0     ,0     ,0     ,0     ,0     ;...
            Ixz/Izz,1     ,0     ,0     ,0     ,0     ,0     ,0     ,0     ,0     ;...
            0     ,0     ,1     ,Ixz/Ixx,0     ,0     ,0     ,0     ,0     ,0     ;...
            0     ,0     ,Ixz/Izz,1     ,0     ,0     ,0     ,0     ,0     ,0     ;...
            0     ,0     ,0     ,0     ,1     ,Ixz/Ixx,0     ,0     ,0     ,0     ;...
            0     ,0     ,0     ,0     ,Ixz/Izz,1     ,0     ,0     ,0     ,0     ;...
            0     ,0     ,0     ,0     ,0     ,0     ,1     ,Ixz/Ixx,0     ,0     ;...
            0     ,0     ,0     ,0     ,0     ,0     ,Ixz/Izz,1     ,0     ,0     ;...
            0     ,0     ,0     ,0     ,0     ,0     ,0     ,0     ,1     ,Ixz/Ixx;...
            0     ,0     ,0     ,0     ,0     ,0     ,0     ,0     ,Ixz/Izz,1    ];
LND = [LB_;NB_;Lp_;Np_;Lr_;Nr_;LDa_;NDa_;LDr_;NDr_];
LN = LNDTLN^(-1)*LND;
YDa = Vto*YDaS;   YDr = Vto*YDrS;  
LB  = LN(1);    Lp  = LN(3);    Lr = LN(5);   LDa = LN(7);    LDr = LN(9);
NB  = LN(2);    Np  = LN(4);    Nr = LN(6);   NDa = LN(8);    NDr = LN(10);
Lv = LB/Vto; Nv = NB / Vto;
%% Stability Derivatives as a Matrix:             
Der0 =[Xu ,0  ,Xw ,0  ,0  ,0  ,0   ,0   ,0   ,XDe ,XDth ;...
       0  ,Yv ,0  ,0  ,0  ,0  ,0   ,YDa ,YDr ,0   ,0    ;...
       Zu ,0  ,Zw ,0  ,Zq ,0  ,ZwD ,0   ,0   ,ZDe ,ZDth ;...
       0  ,Lv ,0  ,Lp ,0  ,Lr ,0   ,LDa ,LDr ,0   ,0    ;...
       Mu ,0  ,Mw ,0  ,Mq ,0  ,MwD ,0   ,0   ,MDe ,MDth ;...
       0  ,Nv ,0  ,Np ,0  ,Nr ,0   ,NDa ,NDr ,0   ,0   ];
Der = [m;m;m;Ixx;Iyy;Izz].* Der0;
%% Initial Gravity Force:
mg0 = m*g * [ sin(s0(8)) ; -cos(s0(8))*sin(s0(7)) ; -cos(s0(8))*cos(s0(7)) ];
FM0 = [mg0;0;0;0];
wdot (1,1) = 0;
%% Loop 
for i = 1:n-1
    % Forces & Moments :
   grav = m*g*[ - sin(y(8,i)) ; cos(y(8,i))*sin(y(7,i)) ; cos(y(8,i))*cos(y(7,i)); 0 ; 0 ; 0 ];
   s     = [y(1:6,i); k1(3); dc ]; %StatesDot(3)
   s_ini = [y(1:6,1); 0    ; 0;0;0;0 ];
   ds = s - s_ini ;
   F_M = (Der * ds) + FM0 + grav ; 
   F = F_M(1:3);
   M = F_M(4:6);
   % Runge Kutta 4 Algorithm:
      k1 = GetStatesDot (y(1:12,i)        ,M,F,I,m);
      k2 = GetStatesDot (y(1:12,i)+dt*k1/2,M,F,I,m);
      k3 = GetStatesDot (y(1:12,i)+dt*k2/2,M,F,I,m);
      k4 = GetStatesDot (y(1:12,i)+dt*k3  ,M,F,I,m);
   y(1:12,i+1) = y(1:12,i)+ dt/6*(k1+2*k2+2*k3+k4);
   y(13:15,i+1) = [ atan(y(3,i)/y(1,i)) ; atan(y(2,i)/y(1,i)) ; sqrt(y(1,i)^2 + y(2,i)^2 + y(3,i)^2) ];
   wdot (1,i+1) = k1(3);
end
u = y(1,:)  ;    v = y(2,:)    ;   w = y(3,:)  ;
p = y(4,:)  ;    q = y(5,:)    ;   r = y(6,:)  ;
phi = y(7,:);    theta = y(8,:);   bsi = y(9,:);
X = y(10,:) ;    Y = y(11,:)   ;   Z = y(12,:) ;
Alpha = y(13,:); Beta = y(14,:); Vtotal = y(15,:);
states = [u;v;w;180/pi*p;180/pi*q;180/pi*r;...
          180/pi*phi;180/pi*theta;180/pi*bsi;X;Y;Z;...
          180/pi*Alpha;180/pi*Beta;Vtotal];
%% LONGTUDINAL
%% Linearized Longtudinal EOM:
A      = [Xu                 ,Xw                 ,-w0                     ,-g*cos(theta0)            ;...
          Zu/(1-ZwD)         ,Zw/(1-ZwD)         ,(Zq+u0)/(1-ZwD)         ,-g*sin(theta0)/(1-ZwD)    ;...
          Mu+ MwD*Zu/(1-ZwD) ,Mw+ MwD*Zw/(1-ZwD) ,Mq+ MwD*(Zq+u0)/(1-ZwD) ,-MwD*g*sin(theta0)/(1-ZwD);...
          0                  ,0                  ,1                       ,0                        ];
B      = [XDe                        ,XDth                 ;...
          ZDe/(1-ZwD)                ,ZDth/(1-ZwD)         ;...
          MDe+MwD*ZDe/(1-ZwD)        ,MDth+MwD*ZDth/(1-ZwD);...
          0                          ,0                    ];
C = [1,0,0,0;...
     0,1,0,0;...
     0,0,1,0;...
     0,0,0,1];         D = [0,0;0,0;0,0;0,0];
% Transfer Function:
[ae,be]  = ss2tf(A,B,C,D,1);     [ath,bth] = ss2tf(A,B,C,D,2);
U_DE     = tf(ae(1,:),be);       U_DTH     = tf(ath(1,:),bth);
W_DE     = tf(ae(2,:),be);       W_DTH     = tf(ath(2,:),bth);
Q_DE     = tf(ae(3,:),be) ;      Q_DTH     = tf(ath(3,:),bth);
THETA_DE = tf(ae(4,:),be);       THETA_DTH = tf(ath(4,:),bth);
%% Short period:
% this approximation will not be used in the remaining parts of the course.
A_short = [Zw/(1-ZwD)                 ,(Zq+ u0)/(1-ZwD)         ;...
          (Mw+ MwD*Zw/(1-ZwD))        , Mq+ MwD*(Zq+u0)/(1-ZwD)];
B_short = [dc(3),dc(4)].*...
          [ZDe/(1-ZwD)                ,ZDth/(1-ZwD)             ;...
           MDe+MwD*ZDe/(1-ZwD)        ,MDth+MwD*ZDth/(1-ZwD)]   ;...
C_short = [1,0;0,1];       DD = [0,0;0,0];
[a_se,b_se] = ss2tf(A_short,B_short,C_short,DD,1);
[a_sth,b_sth] = ss2tf(A_short,B_short,C_short,DD,2);
W_DEs   = tf(a_se(1,:),b_se);   Q_DEs   = tf(a_se(2,:),b_se);
W_DTHs  = tf(a_sth(1,:),b_sth); Q_DTHs  = tf(a_sth(2,:),b_sth);
%% Long period:
% this approximation will not be used in the remaining parts of the course.
A_long = [Xu+ w0*Zu/(Zq+u0)     ,-g*cos(theta0)- w0*g*sin(theta0)/(Zq+u0);...
         -Zu/(Zq + u0)          , g*sin(theta0)/(Zq + u0)               ];
B_long = [dc(3),dc(4)].*[XDe + w0*ZDe/(Zq + u0), XDth  + w0*ZDth/(Zq + u0)              ;...
                        -ZDe/(Zq + u0)         , -ZDth/(Zq + u0)                       ];
[a_le,b_le]   = ss2tf(A_long,B_long,C_short,DD,1);
[a_lth,b_lth] = ss2tf(A_long,B_long,C_short,DD,2);
U_DEl     = tf(a_le(1,:),b_le);    THETA_DEl = tf(a_le(2,:),b_le);
U_DTHl    = tf(a_lth(1,:),b_lth);  THETA_DTHl= tf(a_lth(2,:),b_lth);
%% Longtudinal Autopilot:
servo          = tf(10,[1,10]);
integrator     = tf(1,[1,0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1,[1 0.1]);

OL_theta_thetacom = -servo*THETA_DE;
OL_U_Ucommand = U_DTH*servo*engine_timelag;
h_theta = -integrator*minreal(W_DE/THETA_DE - u0);
OL_h_hCom = minreal(tf(Cl_theta_thetacom)*h_theta);

% theta controller
Z_PI = C1_theta.Z{1};
P_PI = C1_theta.P{1};
K_PI = C1_theta.K;
K_D  = C2_theta.K;

% velocity controller
Z_PIu = C1_u.Z{1};
P_PIu = C1_u.P{1};
K_PIu = C1_u.K;
Z_Du  = C2_u.Z{1};
P_Du  = C2_u.P{1};
K_Du  = C2_u.K;

% altitude controller
K_PIh = C1_h.K;
K_Dh  = C2_h.K;
%% LATERAL
%% Linearized Lateral EOM :
A_Lat = [YB/Vto,(Yp+w0)/Vto,(Yr-u0)/Vto , g*cos(theta0)/Vto,0 ;...
         LB_   , Lp_       , Lr_        , 0                ,0 ;...
         NB_   , Np_       , Nr_        , 0                ,0 ;...
         0     , 1         , tan(theta0), 0                ,0 ;...
         0     , 0         , sec(theta0), 0                ,0];
B_Lat = [YDaS, YDrS   ;...
         LDa_, LDr_   ;...
         NDa_, NDr_   ;...
         0   , 0      ;...
         0   , 0     ];
C_Lat = [1,0,0,0,0;...
         0,1,0,0,0;...
         0,0,1,0,0;...
         0,0,0,1,0;...
         0,0,0,0,1];              D_Lateral = zeros(5,2);
%transfer functions:
[a_a,b_a] = ss2tf(A_Lat,B_Lat,C_Lat,D_Lateral,1);       [a_r,b_r] = ss2tf(A_Lat,B_Lat,C_Lat,D_Lateral,2); 
B_DA      = tf(a_a(1,:),b_a);  B_DR      = tf(a_r(1,:),b_r);
P_DA      = tf(a_a(2,:),b_a);  P_DR      = tf(a_r(2,:),b_r);
R_DA      = tf(a_a(3,:),b_a);  R_DR      = tf(a_r(3,:),b_r);
PHI_DA    = tf(a_a(4,:),b_a);  PHI_DR    = tf(a_r(4,:),b_r);
BSI_DA    = tf(a_a(5,:),b_a);  BSI_DR    = tf(a_r(5,:),b_r);
%% 3 DOF (Spiral)
A_3 = [Lp_,Lr_        ,0;...
       Np_,Nr_        ,0;...
       1  ,tan(theta0),0];
B_3 = [LDa_, LDr_;...
       NDa_, NDr_;...
       0   , 0  ];
C_3 = [1,0,0;...
       0,1,0;...
       0,0,1];    D_3 = zeros(3,2);
%transfer functions:
[a_a3,b_a3] = ss2tf(A_3,B_3,C_3,D_3,1);       [a_r3,b_r3] = ss2tf(A_3,B_3,C_3,D_3,2);
P_DA3   = tf(a_a3(1,:),b_a3);   P_DR3   = tf(a_r3(1,:),b_r3);
R_DA3   = tf(a_a3(2,:),b_a3);   R_DR3   = tf(a_r3(2,:),b_r3);
PHI_DA3 = tf(a_a3(3,:),b_a3);   PHI_DR3 = tf(a_r3(3,:),b_r3);
%% 2 DOF (Dutch Roll)
A_2 = [YB/Vto,(Yr-u0)/Vto - tan(theta0)*(Yp+w0)/Vto ; NB_, Nr_ - tan(theta0)*Np_];
B_2 = [YDaS , YDrS ;NDa_, NDr_];   C_2 = [1,0;0,1];  D_2 = zeros(2,2);
% transfer functions:
[a_a2,b_a2] = ss2tf(A_2,B_2,C_2,D_2,1);       [a_r2,b_r2] = ss2tf(A_2,B_2,C_2,D_2,2);
B_DA2 = tf(a_a2(1,:),b_a2); B_DR2 = tf(a_r2(1,:),b_r2);
R_DA2 = tf(a_a2(2,:),b_a2); R_DR2 = tf(a_r2(2,:),b_r2);
%% 1 DOF (Roll)
A_1 = Lp_;  B_1 = [LDa_,0];  C_1 = 1;  D_1 = [0,0];
% transfer functions:
[a_a1,b_a1] = ss2tf(A_1,B_1,C_1,D_1,1);       [a_r1,b_r1] = ss2tf(A_1,B_1,C_1,D_1,2);
P_DA1 = tf(a_a1(1,:),b_a1); P_DR1 = tf(a_r1(1,:),b_r1);
%% 3 DOF (Dutch Roll)
A_21 = [Yv , 0  , -1   ;...
        LB_, Lp_, Lr_  ;...
        NB_, Np_, Nr_ ]; 
B_21 = [YDaS , YDrS  ;...
        LDa_, LDr_ ;...
        NDa_, NDr_];
% transfer functions:
[a_a21,b_a21] = ss2tf(A_21,B_21,C_3,D_3,1);       [a_r21,b_r21] = ss2tf(A_21,B_21,C_3,D_3,2);
B_DA21 = tf(a_a21(1,:),b_a21);  B_DR21 = tf(a_r21(1,:),b_r21);
P_DA21 = tf(a_a21(2,:),b_a21);  P_DR21 = tf(a_r21(2,:),b_r21);
R_DA21 = tf(a_a21(3,:),b_a21);  R_DR21 = tf(a_r21(3,:),b_r21);
%% Lateral Autopilot:
OL_r_rcom = minreal(servo*R_DR);
yaw_damped_series = feedback(servo*ss(A_Lat,B_Lat,C_Lat,D_Lateral),yaw_damper8,2,3,1);
yaw_damped_tf = tf(yaw_damped_series);
Phi_DA_yawdamped = yaw_damped_tf(4,1);
OL_phi_phicom = minreal(Phi_DA_yawdamped);
%% ALL PLOTS:
% sim ('RBD')
toc
%% Functions  
%% States Dot:
function StatesDot = GetStatesDot (y,M,F,I,m)
u = y(1);       v = y(2);       w = y(3);
p = y(4);       q = y(5);       r = y(6);
phi = y(7);     theta = y(8);   bsi = y(9);
X = y(10);      Y = y(11);      Z = y(12);
StatesDot1= (F/m)-cross([p;q;r],[u;v;w]);
StatesDot2= I\(M - cross([p;q;r],I*[p;q;r]));
StatesDot3= [1,sin(phi)*tan(theta),cos(phi)*tan(theta);...
             0,cos(phi)           ,-sin(phi)          ;...
             0,sin(phi)/cos(theta),cos(phi)/cos(theta)]...
             *[p;q;r];
StatesDot4= eul2rotm([bsi theta phi],'ZYX')*[u ;v ;w];
StatesDot = [StatesDot1;StatesDot2;StatesDot3;StatesDot4];
end
%% States plot:
function  PlotStates(t,states)
chch=['u','v','w','p','q','r',"\phi","\theta","\psi","X","Y","Z","\alpha","\beta","Total Velocity"];
figure (1)
for i=1:15
    subplot (5,3,i)
    plot (t,states(i,:))
    xlabel('time (t)'); ylabel(chch(i));
    grid on; 
    %hold on;
    %plot(ans.tout,states_s(:,i),'--')
    legend(("Matlab"));
end
figure (2)
plot3(states(10,:),states(11,:),states(12,:));
% hold on
% plot3(ans.y_simulink(:,10),ans.y_simulink(:,11),ans.y_simulink(:,12),'--')
title('Trajectory')
xlabel('X'); ylabel('Y'); zlabel('Z');
legend(("Matlab"));
grid on
end
%% Longtudinal Plots:
function LongtudinalPlot(A,B,dc,t,states)
C1 = [1,0,0,0;...
      0,1,0,0;...
      0,0,1,0;...
      0,0,0,1];      D1 = zeros(4,2);        
sys_Long=ss(A,[dc(3),dc(4)].*B,C1,D1);                       [y,~]=step(sys_Long,t);
u_ = y(:,1,:)+ states(1,1);   theta_ = y(:,4,:)+ pi/180*states(8,1);
w_ = y(:,2,:)+ states(3,1);   q_ = y(:,3,:)+ pi/180*states(5,1);    
titless = ["Elevator Command","Thrust Command"] ;
for i = 1:2
figure ('Name',titless(i))
subplot(2,2,1)
plot(t,u_(:,:,i),t,states(1,:))
title('u')
legend("linear","Matlab");
grid on
subplot(2,2,2)
plot(t,w_(:,:,i),t,states(3,:))
title('w')
legend("linear","Matlab");
grid on
subplot(2,2,3)
plot(t,180/pi*q_(:,:,i),t,states(5,:))
title('q')
legend("linear","Matlab");
grid on
subplot(2,2,4)
plot(t,180/pi*theta_(:,:,i),t,states(8,:))
title('\theta')
legend("linear","Matlab");
grid on
end
end
function FrequencyLongtudinal(U_DE,W_DE,Q_DE,THETA_DE,U_DTH,W_DTH,Q_DTH,THETA_DTH)
Long_sys   = [U_DE ;W_DE ;Q_DE ;THETA_DE ;U_DTH ;W_DTH ;Q_DTH ;THETA_DTH ];
Long_sys_ch= ["u / \delta_{Elevator}" ,"w / \delta_{elevator}" ,"q / \delta_{Elevator}" ,"\theta / \delta_{Elevator}" ,...
              "u / \delta_{Thrust}" ,"w / \delta_{Thrust}","q / \delta_{Thrust}","\theta / \delta_{Thrust}" ];
%long_short = [U_DEl;W_DEs;Q_DEs;THETA_DEl;U_DTHl;W_DTHs;Q_DTHs;THETA_DTHl];
% Root locus:
figure ('Name',"Root Locus")
for i = 1:8
subplot(2,4,i)
rlocus(Long_sys(i))
title(Long_sys_ch(i))
grid
end
% Bode plots:
figure ('Name',"Bode Plots")
for i = 1:8
subplot(2,4,i)
bode(Long_sys(i))
title(Long_sys_ch(i))
grid
end
end
%% Longtudinal Approximations Plots:
function LongtudinalApproxPlot(A_short,B_short,A_long,B_long,dc,t,states)
C2 = [1,0;0,1];   D2 = zeros(2,2);
sys_Long_period=ss(A_long,[dc(3),dc(4)].*B_long,C2,D2);      [yl,~]=step(sys_Long_period,t);
sys_short_period=ss(A_short,[dc(3),dc(4)].*B_short,C2,D2);   [ys,~]=step(sys_short_period,t);
ul = yl(:,1,:)+states(1,1);     thetal =yl(:,2,:)+ pi/180*states(8,1);
ws =ys(:,1,:)+ states(3,1);     qs =ys(:,2,:)+ pi/180*states(5,1);  
titless=["Elevator Command","Thrust Command"];
for i = 1:2
figure('Name',titless(i))
subplot(2,2,1)
plot(t,ul(:,:,i),t,states(1,:))
title('u')
legend("Linear Approx.","Matlab");
grid on
subplot(2,2,2)
plot(t,ws(:,:,i),t,states(3,:))
title('w')
legend("Linear Approx.","Matlab");
grid on
subplot(2,2,3)
plot(t,180/pi*qs(:,:,i),t,states(5,:))
title('q')
legend("Linear Approx.","Matlab");
grid on
subplot(2,2,4)
plot(t,180/pi*thetal(:,:,i),t,states(8,:))
title('\theta')
legend("Linear Approx.","Matlab");
grid on
end
end
function LongtudinalApproxFrequency(U_DEl,W_DEs,Q_DEs,THETA_DEl,U_DTHl,W_DTHs,Q_DTHs,THETA_DTHl)
long_short = [U_DEl;W_DEs;Q_DEs;THETA_DEl;U_DTHl;W_DTHs;Q_DTHs;THETA_DTHl];
Long_sys_ch= ["u / \delta_{Elevator}" ,"w / \delta_{elevator}" ,"q / \delta_{Elevator}" ,"\theta / \delta_{Elevator}" ,...
              "u / \delta_{Thrust}" ,"w / \delta_{Thrust}","q / \delta_{Thrust}","\theta / \delta_{Thrust}" ];
% Root locus:
figure ('Name',"Root Locus")
for i = 1:8
subplot(2,4,i)
rlocus(long_short(i))
title(Long_sys_ch(i))
grid
end
% Bode plots:
figure ('Name',"Bode Plots")
for i = 1:8
subplot(2,4,i)
bode(long_short(i))
title(Long_sys_ch(i))
grid
end
end
%% Lateral Plots:
function LateralPlot(A_Lat,B_Lat,C_Lat,D_Lateral,dc,t,states)
sys_Lat = ss(A_Lat,[dc(1),dc(2)].*B_Lat,C_Lat,D_Lateral);
[y,~]= step(sys_Lat,t);   titless = ["Aileron Command","Rudder Command"];
B= y(:,1,:)+ pi/180*states(14,1);  p = y(:,2,:)+pi/180*states(4,1);  r= y(:,3,:)+pi/180*states(6,1);  phi= y(:,4,:)+pi/180*states(7,1);  
for i = 1:2
figure('Name',titless(i))
subplot(2,2,1)
plot(t,180/pi*B(:,:,i),t,states(14,:))
title('\beta')
legend("linear","Matlab");
grid on
subplot(2,2,2)
plot(t,180/pi*p(:,:,i),t,states(4,:))
title('p')
legend("linear","Matlab");
grid on
subplot(2,2,3)
plot(t,180/pi*r(:,:,i),t,states(6,:))
title('r')
legend("linear","Matlab");
grid on
subplot(2,2,4)
plot(t,180/pi*phi(:,:,i),t,states(7,:))
title('\phi')
legend("linear","Matlab");
grid on
end
end
function LateralFrequency(B_DA,P_DA,R_DA,PHI_DA,BSI_DA,B_DR,P_DR,R_DR,PHI_DR,BSI_DR)
Lateral_sys = [B_DA ;P_DA ;R_DA ;PHI_DA ;BSI_DA ;B_DR ;P_DR ;R_DR ;PHI_DR ;BSI_DR ];
% Root locus:
figure ('Name',"Root Locus")
for i = 1:10
subplot(2,5,i)
rlocus(Lateral_sys(i))
grid
end
% Bode Plot:
figure ('Name',"Bode Plot")
for i = 1:10
subplot(2,5,i)
bode(Lateral_sys(i))
grid
end
end
%% Lateral Approximations Plots:
function LateralApproxPlot(A_3,A_2,A_1,A_21,B_3,B_2,B_1,B_21,C_3,C_2,C_1,D_3,D_2,D_1,dc,t,states)
% 3
sys_3lat = ss(A_3,[dc(1),dc(2)].*B_3,C_3,D_3);
[y_3,~]= step (sys_3lat,t);
p3 = y_3(:,1,:)+ pi/180*states(4,1); r3 = y_3(:,2,:)+pi/180*states(6,1); phi3 = y_3(:,3,:)+pi/180*states(7,1);
% 2
sys_2lat = ss(A_2,[dc(1),dc(2)].*B_2,C_2,D_2);
[y_2,~]= step (sys_2lat,t);
B2 = y_2(:,1,:)+pi/180*states(14,1);   r2 = y_2(:,2,:)+pi/180*states(6,1);
% 1
sys_2lat = ss(A_1,[dc(1),dc(2)].*B_1,C_1,D_1);
[y_1,~]= step (sys_2lat,t);   p1 = y_1(:,1,:)+pi/180*states(4,1);
% 2+1
sys_21_lat = ss(A_21,[dc(1),dc(2)].*B_21,C_3,D_3);
[y_21,~]= step (sys_21_lat,t);
B21 = y_21(:,1,:)+pi/180*states(14,1);  p21 = y_21(:,2,:)+pi/180*states(4,1);  r21 = y_21(:,3,:)+pi/180*states(6,1);
titless=["Aileron Command","Rudder Command"];
for i = 1:2
figure('Name',titless(i))
subplot(2,2,1)
plot(t,180/pi*B2(:,:,i),t,180/pi*B21(:,:,i),t,states(14,:))
title('\beta')
legend("2 DOF","3 DOF Dutch Roll","Matlab");
grid on
subplot(2,2,2)
plot(t,180/pi*p3(:,:,i),t,180/pi*p1(:,:,i),t,180/pi*p21(:,:,i),t,states(4,:))
title('p')
legend("3 DOF Spiral","1 DOF","3 DOF DUtch Roll","Matlab");
grid on
subplot(2,2,3)
plot(t,180/pi*r3(:,:,i),t,180/pi*r2(:,:,i),t,180/pi*r21(:,:,i),t,states(6,:))
title('r')
legend("3 DOF Spiral","2 DOF","3 DOF Dutch Roll","Matlab");
grid on
subplot(2,2,4)
plot(t,180/pi*phi3(:,:,i),t,states(7,:))
title('\phi')
legend("3 DOF Spiral","Matlab");
grid on
end
end
function LateralApproxFrequency(P_DA3,R_DA3,PHI_DA3,P_DR3,R_DR3,PHI_DR3,B_DA2,R_DA2,B_DR2,R_DR2,P_DA1,P_DR1,B_DA21,P_DA21,R_DA21,B_DR21,P_DR21,R_DR21)
DOF3  = [P_DA3 ;R_DA3 ;PHI_DA3 ;P_DR3 ;R_DR3 ;PHI_DR3];
DOF2  = [B_DA2 ;R_DA2 ;B_DR2 ;R_DR2];    DOF1  = [P_DA1 ;P_DR1 ];
DOF21 = [B_DA21 ;P_DA21 ;R_DA21 ;B_DR21 ;P_DR21;R_DR21];
figure ('Name',"Root Locus")
for i = 1:6
subplot(2,3,i)
rlocus(DOF3(i))
grid
end
figure ('Name',"Root Locus")
for i = 1:4
subplot(2,2,i)
rlocus(DOF2(i))
grid
end
figure ('Name',"Root Locus")
for i = 1:2
subplot(2,1,i)
rlocus(DOF1(i))
grid
end
figure ('Name',"Root Locus")
for i = 1:6
subplot(2,3,i)
rlocus(DOF21(i))
grid
end
figure ('Name',"Bode Plot")
for i = 1:6
subplot(2,3,i)
bode(DOF3(i))
grid
end
figure ('Name',"Bode Plot")
for i = 1:4
subplot(2,2,i)
bode(DOF2(i))
grid
end
figure ('Name',"Bode Plot")
for i = 1:2
subplot(2,1,i)
bode(DOF1(i))
grid
end
figure ('Name',"Bode Plot")
for i = 1:6
subplot(2,3,i)
bode(DOF21(i))
grid
end
end