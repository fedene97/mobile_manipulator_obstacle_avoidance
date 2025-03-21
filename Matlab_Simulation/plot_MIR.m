function plot_MIR(Q,r)
global mir i UR5 X_ac_nav distancex h_mir Dis_ruote

xnav = X_ac_nav(1,i);
ynav = X_ac_nav(2,i);
yaw = X_ac_nav(6,i);

InitPoseKMR = [0; 0; 0];

x0 = 0;
y0 = 0;
z0 = 0;

Tbase = [ cos(yaw) -sin(yaw) 0 xnav+x0*cos(yaw)-y0*sin(yaw);
          sin(yaw)  cos(yaw) 0 ynav+x0*sin(yaw)+y0*cos(yaw);
                 0         0 1                            0;
                 0         0 0                            1];

%% plot mobile robot

mir_Points=mir.Points;
mir_Points(:,1)=-mir.Points(:,1);
mir_Points(:,2)=-mir.Points(:,2);

mir_Points(:,1)=mir_Points(:,1)+(max(mir_Points(:,1))-min(mir_Points(:,1)))/2-max(mir_Points(:,1));
mir_Points(:,1)=mir_Points(:,1)+Dis_ruote;
mir_Points(:,2)=mir_Points(:,2)+(max(mir_Points(:,2))-min(mir_Points(:,2)))/2-max(mir_Points(:,2));
mir_Points(:,3)=mir_Points(:,3)-min(mir_Points(:,3));

M1 = ones(size(mir_Points,1),3);
InitPoseKMR = M1.*InitPoseKMR';
PosizioneKMR = (mir_Points/1000)+InitPoseKMR;
RotazioneKMR = (eye(3,4)*Tbase*[PosizioneKMR ones(size(PosizioneKMR,1),1)]')';

c = ones(size(mir_Points,1),3);
% Grey colour
idx = find(mir_Points(:,3)<180);
c(idx,1)=0.4;
c(idx,2)=0.4;
c(idx,3)=0.4;

% Blue colour
idx = find(mir_Points(:,3)>=180 & mir_Points(:,3)<200);
c(idx,1)=0;
c(idx,2)=0.9;
c(idx,3)=1;

% Grey colour
idx = find(mir_Points(:,3)>=200);
c(idx,1)=0.4;
c(idx,2)=0.4;
c(idx,3)=0.4;

trisurf(triangulation(mir.ConnectivityList,RotazioneKMR),'facevertexcdata',[c(:,1) c(:,2) c(:,3)],'FaceColor','interp','edgealpha',0);

hold on

%% plot manipulator
p0 = [xnav + distancex*cos(yaw), ynav + distancex*sin(yaw), h_mir, yaw];
show(UR5,Q(4:9)', 'Frames','off','position',p0);
camHandle = camlight;
set(camHandle,'Visible','on')

end
