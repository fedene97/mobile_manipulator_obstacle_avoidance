function [X, points, nav, nav2]=kin_dir_UR5_Link(Q)
global tool X_ac i alphapunta X_i X_ac_nav distancex h_mir r_mir l_mir d_mir r_min

x0 = 0;
y0 = 0;
z0 = 0;

xnav = X_ac_nav(1,i);
ynav = X_ac_nav(2,i);
yaw = X_ac_nav(6,i);

Tbase = [ cos(yaw) -sin(yaw) 0 xnav+x0*cos(yaw)-y0*sin(yaw);
          sin(yaw)  cos(yaw) 0 ynav+x0*sin(yaw)+y0*cos(yaw);
                 0         0 1                            0;
                 0         0 0                            1];

Twb = Tbase;
Twb(1,4) = Twb(1,4) + distancex*cos(yaw);
Twb(2,4) = Twb(2,4) + distancex*sin(yaw);
R_0=Twb(1:3,1:3);
p_0=Twb(1:3,4);

[a_2,a_3,d_1,d_4,d_5,d_6,s_1,s_3]=Misure_simulazione();

q_1=Q(4); q_2=Q(5);q_3=Q(6);q_4=Q(7);q_5=Q(8);q_6=Q(9);

 % A
 R_1=[cos(q_1)       -sin(q_1)         0;
      sin(q_1)        cos(q_1)         0;
      0                 0              1];
 
 % B
 R_2=[-sin(q_2)     0        cos(q_2);
      0            1              0 ;
     -cos(q_2)     0        -sin(q_2)];

 % C
 R_3=[cos(q_3)     0        sin(q_3);
      0            1            0   ;
      -sin(q_3)    0        cos(q_3)];
 
 % B1
 R_B1=[1          0           0;
       0          1           0;
       0          0           1];

 % D
R_4=[-sin(q_4)     0          +cos(q_4);
    0             1             0    ;
    -cos(q_4)      0         -sin(q_4)];

 % E
 R_5=[cos(q_5)    -sin(q_5)     0;
     sin(q_5)      cos(q_5)     0;
       0              0         1]; 
 
 % F
 R_6=[cos(q_6)      0        sin(q_6);
     0              1              0 ;
     -sin(q_6)      0        cos(q_6)];

 R_7=[  0   -1    0;
        0    0    1;
       -1    0    0];

p_1=[0;0;d_1];                                             % A
p_2=[0;s_1;0];                                             % B
p_3=[0;0;a_2/3];                                           % C
p_4=[0;0;a_3/3];                                           % D
p_5=[0;d_4;0];                                             % E  
p_6=[0;0;d_5];                                             % F
p_7=[0;d_6;0];                                             % G 

T_0 = Twb;
T_0(3,4) = h_mir;
T_1 = [R_1,p_1;zeros(1,3),1];
T_2 = [R_2,p_2;zeros(1,3),1];
T_3 = [R_3,p_3;zeros(1,3),1];
T_4 = [R_4,p_4;zeros(1,3),1];
T_5 = [R_5,p_5;zeros(1,3),1];
T_6 = [R_6,p_6;zeros(1,3),1];

% A (joint 1)
T_01=T_0*T_1;
A(1:3)=T_01(1:3,4);
A(4:6)=ang_eulero(T_01(1:3,1:3));

% B (joint 2)
T_02=T_01*T_2;
B(1:3)=T_02(1:3,4);
B(4:6)=ang_eulero(T_02(1:3,1:3));

% B1 (sul link 2)
p_B1=[0;0;a_2/3];
T_B1=[R_B1,p_B1;zeros(1,3),1];
T_0B1=T_02*T_B1;
B1(1:3)=T_0B1(1:3,4);
B1(4:6)=ang_eulero(T_0B1(1:3,1:3));

% B2 (sul link 2)
p_B2=[0;0;a_2/3];
T_B2=[R_B1,p_B2;zeros(1,3),1];
T_0B2=T_0B1*T_B2;
B2(1:3)=T_0B2(1:3,4);
B2(4:6)=ang_eulero(T_0B1(1:3,1:3));

% C (joint 3)
T_03=T_0B2*T_3;
C(1:3)=T_03(1:3,4);
C(4:6)=ang_eulero(T_03(1:3,1:3));

% C1 (between C and D)
p_c1=[0;-s_3;0];
R_c1=R_B1;
T_c1=[R_c1,p_c1;zeros(1,3),1];
T_0c1=T_03*T_c1;
C1(1:3)=T_0c1(1:3,4);
C1(4:6)=ang_eulero(T_0c1(1:3,1:3));

% C2 (between C and D)
p_C2=[0;0;a_3/3]; 
T_C2=[R_B1,p_C2;zeros(1,3),1];
T_0C2=T_0c1*T_C2;
C2(1:3)=T_0C2(1:3,4);
C2(4:6)=ang_eulero(T_0C2(1:3,1:3));

% C3 (between C and D)
p_C3=[0;0;a_3/3]; 
T_C3=[R_B1,p_C3;zeros(1,3),1];
T_0C3=T_0C2*T_C3;
C3(1:3)=T_0C3(1:3,4);
C3(4:6)=ang_eulero(T_0C3(1:3,1:3));

% D (joint 4)
T_04=T_0C3*T_4;
D(1:3)=T_04(1:3,4);
D(4:6)=ang_eulero(T_04(1:3,1:3));

% E (joint 5) 
T_05=T_04*T_5;
E(1:3)=T_05(1:3,4);
E(4:6)=ang_eulero(T_05(1:3,1:3));

% F (joint 6) 
T_06=T_05*T_6;
F(1:3)=T_06(1:3,4);
F(4:6)=ang_eulero(T_06(1:3,1:3));
 
% G (end effector)
T_7=[R_7,p_7;zeros(1,3),1];
T_07=T_06*T_7;
G(1:3)=T_07(1:3,4);
G(4:6)=ang_eulero(T_07(1:3,1:3));
 
% Dnav
Dnav = eye(3,4)*(Twb*[0 0 0 1]');
D1nav  = [Dnav' (ang_eulero(Twb(1:3,1:3)))]';
Dnav = D1nav+[0 0 h_mir 0 0 0]';
 
X=G';

points=[[(eye(3,4)*(Twb*[0 0 0 1]'))' 0 0 0]' A' B' B1' B2' C' C1' C2' C3' D' E' F' G' D1nav Dnav];

nav = [-l_mir/2 -l_mir/2   l_mir/2  (d_mir/2)*tand(alphapunta)+(l_mir/2)    l_mir/2;
       -d_mir/2  d_mir/2   d_mir/2                             0   -d_mir/2;
          0    0     0                             0      0;
          1    1     1                             1      1];

nav = Tbase*[nav-[0 0 h_mir 0]'];

nav2 = [-l_mir/2-r_min -l_mir/2-r_min   l_mir/2+r_min  (d_mir/2)*tand(alphapunta)+(l_mir/2)+r_min    l_mir/2+r_min;
        -d_mir/2-r_min  d_mir/2+r_min   d_mir/2+r_min                                           0   -d_mir/2-r_min;
          0    0     0                             0      0;
          1    1     1                             1      1];

nav2 = Tbase*[nav2-[0 0 h_mir 0]'];
end


 