clc
clear
close all
clear global

global X_i X_f dt T r r_min r_inf r_sup Data...
       v0_rep v_1 v_2 v0_att k_e Q_i Q_i_KMR O dO k_v lambda_max eps...
       UR5 ik weights tool esempio K_path k_e_tra k_e_rot k_e_rotz K1 K2 K3...
       dQ_max dQ_max_g mir X_ac dX_ac i alphapunta X_ac_nav A_J A_J_diag distancex h_mir r_mir l_mir d_mir Dis_ruote

%% Start angle=end angle along the trajectory
% If 0 takes the entered start and end angle
% If 1 aligns the angle on the line joining the start and end points
align= 0;

%% Data
esempio = 3; % 1 2 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Example
[v_1,v_2,Oi,Of,dOi,X_i,X_f]=Dati_simulazione(esempio);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dQ_max=[0.5,0.5,pi/2]; % m/s
dQ_max_g=pi/2;
r_min=0.10;
r=4/3*r_min;
v0_att=1;
k_e=[k_e_tra,k_e_tra,k_e_tra,k_e_rot,k_e_rot,k_e_rotz]';
steps=500; % 500 is better
dt=T/steps;
k_v=0;
order_Bezier_curve = 3;
lambda_max=10^-3;
eps=10^-3;
r_inf = r_min;
r_sup = r;
alphapunta = 40;
tool= 0;
Data = 'Dati'; % folder name
Frame_count = 50;

A_J_mir = 1;
A_J_ur = 1;
A_J_diag = [A_J_mir A_J_mir A_J_mir A_J_ur A_J_ur A_J_ur A_J_ur A_J_ur A_J_ur A_J_ur];
A_J = diag(A_J_diag);% weighted pseudoinverse matrix
for j=1:10 % lock joints
    if diag(A_J(j,j))==0 || diag(A_J(j,j))>10^15
        A_J(j,j)=1;
    end
end

%% MIR data
DownUR = 0.028; % UR decrease compared to the plan
h_mir = 0.355 + 0.340 - DownUR;
Dis_ruote = 33; % mm
distancex = -0.26 + Dis_ruote/1000; % Misalignment of the arm from the center
r_mir = 0.010;
l_mir = 0.890;
d_mir = 0.580;

%% Color MIR
mir = stlread('ARSI_Chimera_v03.stl');
mir_Points=mir.Points;
mir_Points(:,1)=-mir.Points(:,1);
mir_Points(:,2)=-mir.Points(:,2);

mir_Points(:,1)=mir_Points(:,1)+(max(mir_Points(:,1))-min(mir_Points(:,1)))/2-max(mir_Points(:,1));
mir_Points(:,1)=mir_Points(:,1)+Dis_ruote;
mir_Points(:,2)=mir_Points(:,2)+(max(mir_Points(:,2))-min(mir_Points(:,2)))/2-max(mir_Points(:,2));
mir_Points(:,3)=mir_Points(:,3)-min(mir_Points(:,3));

c = ones(size(mir_Points,1),3);
% Gray color
idx = find(mir_Points(:,3)<180);
c(idx,1)=0.4;
c(idx,2)=0.4;
c(idx,3)=0.4;

% Blue color
idx = find(mir_Points(:,3)>=180 & mir_Points(:,3)<200);
c(idx,1)=0;
c(idx,2)=0.9;
c(idx,3)=1;

% Gray color
idx = find(mir_Points(:,3)>=200);
c(idx,1)=0.4;
c(idx,2)=0.4;
c(idx,3)=0.4;

trisurf(triangulation(mir.ConnectivityList,mir_Points/1000),'facevertexcdata',[c(:,1) c(:,2) c(:,3)],'FaceColor','interp','edgealpha',0);
camhandle = camlight;
set(camhandle,'visible','on')
material('dull')
ImpostazioneView % Set display options

%% Robot
UR5 = importrobot('UR10_correct.urdf');
UR5.DataFormat='row';
ik = robotics.InverseKinematics('RigidBodyTree', UR5);
weights = [1, 1, 1, 1, 1, 1];


%% Inverse position kinematics
R_ad=[0    0    -1;
      0    1     0;
      1    0     0];

eul_i=X_i(4:6); % Euler angles
eul_f=X_f(4:6); % Euler angles
R_i=eul2rotm(eul_i','ZYZ')*(R_ad); % rotation matrix in the initial configuration of the manipulator (initial configuration is that of the .urdf model)
R_f=eul2rotm(eul_f','ZYZ')*(R_ad);
X_model_i=X_i-[distancex*cos(X_i(6));distancex*sin(X_i(6));h_mir;0;0;0];
X_model_f=X_f-[distancex*cos(X_i(6));distancex*sin(X_i(6));h_mir;0;0;0];
X_model_i(4:6)=rotm2eul(R_i,'ZYZ');
X_model_f(4:6)=rotm2eul(R_f,'ZYZ');

%% Starting configuration
initialguess=[-pi -pi/2 0 -pi/2 pi/2 0];
pose=[eul2rotm(X_model_i(4:6)','ZYZ') X_model_i(1:3); 0 0 0 1];
Q_i = ik('ee_link',pose,weights,initialguess);
Q_i=Q_i';
pose=[eul2rotm(X_model_f(4:6)','ZYZ') X_model_f(1:3); 0 0 0 1];
Q_f = ik('ee_link',pose,weights,initialguess);
Q_f=Q_f';

%% If i want initial angle = final angle along the trajectory
if align== 1
    difftan = X_f(1:2)-X_i(1:2);
    X_i(6) = atan2(difftan(2),difftan(1));
    X_f(6) = X_i(6);
end

%% Program start
Q_i_KMR = [0 0 X_i(6) Q_i' 0]';

%% Test plot
i=1;
X_ac_nav=[0;0;0;0;0;0];
plot_MIR(Q_i_KMR,r_min); hold on
plot_UR5_link(Q_i_KMR,0.1);
axis equal

%% Calculate straight trajectory for obstacle, to simulate moving obstacles
[O,dO]=obstacles_generation(Oi,Of,dOi,dt,T);

%% Calculate a point-to-point trajectory, without considering obstacles
pppO=O; % to ignore the obstacles
pppdO=dO;
O=O*10000;
dO=dO*10000;
[t,X,dX,X_bez,dX_bez]=trajectory3D(order_Bezier_curve); % Calculate trajectory

%% 3D trajectory plot
figure(2)
plot_sfere(O, 0.05)
plot3(X(1,:),X(2,:), X(3,:),'.g')
plot3(X_bez(1,:),X_bez(2,:), X_bez(3,:),'.m')
axis equal
xlim ([min(X_i(1),X_f(1))-1.2 max(X_i(1),X_f(1))+1.2])
ylim ([min(X_i(2),X_f(2))-1.2 max(X_i(2),X_f(2))+1.2])
zlim ([-0.1 1.9])
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
view(0,90);

X = X_bez;
dX = dX_bez;
O=pppO;
dO=pppdO;

%% Planning considering non-holonomic constraints
out = moto(0,0,0,1,0,0,0,1); % fifth order polynomial law
s=out(:,2)'; 
Iab=s*T;
t=Iab;

dxi = K_path*cos(X_i(6));
dyi = K_path*sin(X_i(6));
dxf = K_path*cos(X_f(6));
dyf = K_path*sin(X_f(6));
alpha = [K_path*cos(X_f(6))-3*X_f(1);
         K_path*sin(X_f(6))-3*X_f(2)];
beta = [K_path*cos(X_i(6))+3*X_i(1);
        K_path*sin(X_i(6))+3*X_i(2)];
n = 1;
for s = Iab/T
    X_nav(1,n) = -(s-1)^3*X_i(1)+s^3*X_f(1)+alpha(1)*s^2*(s-1)+beta(1)*s*(s-1)^2;
    X_nav(2,n) = -(s-1)^3*X_i(2)+s^3*X_f(2)+alpha(2)*s^2*(s-1)+beta(2)*s*(s-1)^2;
    X_nav(3:5,n) = 0;
    if s == 0
        X_nav(6,n) = X_i(6);
    elseif s == 1
        X_nav(6,n) = X_f(6);
    else
        X_nav(6,n) = atan2(X_nav(2,n)-X_nav(2,n-1),X_nav(1,n)-X_nav(1,n-1));
    end
    n = n + 1;
end

dX_nav = [zeros(6,1) diff(X_nav')']/dt;

%% trajectory plot
figure(200)
tiledlayout(2,2);
nexttile
plot3(X_nav(1,:),X_nav(2,:), X_nav(3,:),'.b')
axis equal
hold on
xlim ([min(X_i(1),X_f(1))-0.5 max(X_i(1),X_f(1))+0.5])
ylim ([min(X_i(2),X_f(2))-0.5 max(X_i(2),X_f(2))+0.5])
zlim ([-1 1])
P=X_i(1:3,1);
eul=X_i(4:6,1);
R= eul2rotm(eul','ZYZ');
quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.2,'r')
quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.2,'g')
P=X_f(1:3,1);
eul=X_f(4:6,1);
R= eul2rotm(eul','ZYZ');
quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.2,'r')
quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.2,'g')
xlabel('x [m]', 'Interpreter','latex')
ylabel('y [m]', 'Interpreter','latex')
zlabel('z [m]', 'Interpreter','latex')
view(0,90);

nexttile
hold on
plot(t,rad2deg(X(6,:)),'.b');
xlabel('t [s]', 'Interpreter','latex')
ylabel('$\theta [deg]$', 'Interpreter','latex')
xlim ([0 T])

nexttile
hold on
plot(t,dX_nav(1:2,:));
plot(t,dX_nav(6,:));
xlabel('t [s]', 'Interpreter','latex')
ylabel('velocity', 'Interpreter','latex')
legend('$\dot{x}$','$\dot{y}$','$\dot{\theta}$','Location','northoutside','NumColumns',3, 'Interpreter','latex')
xlim ([0 T])

nexttile
hold on
plot(t,'.b');
xlabel('steps', 'Interpreter','latex')
ylabel('t [s]', 'Interpreter','latex')
xlim ([0 steps])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Inverse kinematics calculation
[Q,distanza_minima,dQ]=kin_inv_OA_KMR(t,X,dX,O,dO,X_nav,dX_nav);

figure(1000)
plot(t(2:end),1000*distanza_minima,'b');hold on
plot([0 T],1000*[r r],'--k')
plot([0 T],1000*[r_min r_min],'--k')
plot([0 T],1000*[(r_min+r)/2 (r_min+r)/2],'--k')
xlim([0 T])
ylim([1000*r_min 1000*max(distanza_minima)+5])
xlabel('t [s]')
ylabel ('[mm]')
legend('minimum distance robot-obstacle')
saveas(gcf,append('.\',Data,'\test_',char(datetime('now','format','yyMMdd_HHmmss')),'OA_dist.png'));
span=10;

%% Start simulation plot
vidName = ['.\',Data,'\test_',char(datetime('now','format','yyMMdd_HHmmss')),'.mp4'];
fig2 = figure(20);
fig2.WindowState = 'maximized';
vid = VideoWriter(vidName,'MPEG-4');
open(vid);
view(-45,30)
camva(9);
axis equal
traiettoria=[];
n_frame=5;
Frame_i = 0;
for i=1:n_frame:size(Q,2)
cla

[X_test,punti] = kin_dir_UR5_Link(Q(:,i));
traiettoria=[traiettoria X_test(1:3)];
hold on
plot3(traiettoria(1,:),traiettoria(2,:),traiettoria(3,:),'k','linewidth',1);
plot3(X(1,:),X(2,:),X(3,:),'--k','linewidth',1); % Planned trajectory
plot_MIR(Q(:,i),r_min);
plot_sfere(O(:,:,i), r_min/4)
P=X_i(1:3,1);
eul=X_i(4:6,1);
R= eul2rotm(eul','ZYZ');
quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.1,'r')
quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.1,'g')
quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),0.1,'b')
P=X_f(1:3,1);
eul=X_f(4:6,1);
R= eul2rotm(eul','ZYZ');
quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.1,'r')
quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.1,'g')
quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),0.1,'b')
axis equal
xlim ([min(X_i(1),X_f(1))-1.5 max(X_i(1),X_f(1))+1.5])
ylim ([min(X_i(2),X_f(2))-1.5 max(X_i(2),X_f(2))+1.5])
zlim ([-0 2.0])
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
view(60,20);
F(i)= getframe;
frame = getframe(gcf);
grid on

% Frame_i = Frame_i + 1;
% if i==1
%     saveas(gcf,append('.\Immagini\','Cad',num2str(0),'.png'));
%     print(gcf,append('.\Immagini\','Cad',num2str(0),'.eps'),'-depsc');
% end
% if Frame_i*n_frame>=Frame_count
%     saveas(gcf,append('.\Immagini\','Cad',num2str(i+n_frame-1),'.png'));
%     print(gcf,append('.\Immagini\','Cad',num2str(i+n_frame-1),'.eps'),'-depsc');
%     Frame_i = 0;
% end
% 
% writeVideo(vid,frame);
end

close(vid);