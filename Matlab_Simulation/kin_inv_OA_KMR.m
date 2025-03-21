function [Q,distanza_minima,dQ_vect] = kin_inv_OA_KMR(t,X,dX,O,dO,X_nav,dX_nav)
global  esempio dt  r r_min r_inf r_sup v_1 v_2 v0_rep k_e Data Q_i Q_i_KMR k_v lambda_max eps dQ_max dQ_max_g X_ac dX_ac i X_i X_f X_ac_nav A_J A_J_diag distancex h_mir r_mir l_mir d_mir K1 K2 K3 
tolleranza=0.90;
Q(:,1)=Q_i_KMR;
X_ac = X_i;
dQ_vect(:,1)=0*Q_i_KMR;
X_ac_nav = [0 0 0 0 0 X_i(6)]'; %% Assume initial base in 0
fail=0;
Frame_i = 0;
Frame_count=50;

%% Start simulation plot
vidName2 = ['.\',Data,'\test_',char(datetime('now','format','yyMMdd_HHmmss')),'.mp4'];
vid2 = VideoWriter(vidName2,'MPEG-4');
open(vid2);

for i=1:size(t,2)-1

    %% select obstacle and respective speed
    o = O(:,:,i);
    V_o = dO(:,:,i);

    %% planned cartesian coordinate
    X_pl = X(:,i);
    
    %% time step
    dt = t(i+1)-t(i);

    distEndPoint = norm(X_f(1:3)-X_ac(1:3,i));
    if distEndPoint>=3.0
        zona = 1;
        A_J_mir = 1;
        A_J_ur = 1;
    elseif distEndPoint>=1.5
        zona = 2;
        A_J_mir = 10^(-5);
        A_J_ur = 10^5;
    else
        zona = 3;
        A_J_mir = 1;
        A_J_ur = 1;
    end
    A_J_diag = [A_J_mir A_J_mir A_J_mir A_J_ur A_J_ur A_J_ur A_J_ur A_J_ur A_J_ur A_J_ur];
    A_J = diag(A_J_diag); % weighted pseudoinverse matrix
    
    %% OA mobile manipulator
    %% J end effector calculation
    J = Jacobian_KMR(Q(:,i),16,1); % end effector jacobian
    S = svd(J);
    s_min = S(6);
    if s_min<eps
        lambda_quadro = (1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda_quadro = 0;
    end

    %% pseudo-inverse right calculation of partial J
    pinvd_J = inv(A_J)*J'*inv(J*inv(A_J)*J'+lambda_quadro*eye(6));

    %% select translation lines
    J_par = J(1:3,:);
    S = svd(J_par);
    s_min = S(3);
    if s_min<eps
        lambda_quadro = (1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda_quadro = 0;
    end

    %% pseudo-inverse right calculation of partial J
    pinvd_J_par = inv(A_J)*J_par'*inv(J_par*inv(A_J)*J_par'+lambda_quadro*eye(3));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% for the current configuration I calculate the position of the points
    [~,punti_robot,nav,nav2] = kin_dir_UR5_Link(Q(:,i));
    punti_robot = punti_robot(1:3,:);

    %% looking for an obstacle closer to the centre of the robot
    distCentro=0;
    for m=1:size(o,2)
        distCentro(m) = norm(o(1:2,m)-X_ac_nav(1:2,i));
    end
    [M,I]=min(distCentro);
    nearO=o(:,I);

    %% Adding segments for the mobile robot
        punti_robot = [punti_robot nav(1:3,:)];
        % set the dimension of the segments to that of the obstacle
        if nearO(3)<= h_mir || zona==1
            punti_robot(3,16:end) = nearO(3);
        else
            punti_robot(3,16:end) = 0;
        end
        corr = [15 2   2 3   3 6   6 7   7 10   10 11   11 12   12 13   (15+[1 2   2 3   3 4   4 5   5 1])]; % rectangle with point, 5 points

        segm = punti_robot(1:3,corr);

    %% find the minimum distance of the obstacle from each link
    [k,dist,x]=dis_link_mir(segm,o); % find the minimum distance of the obstacle from each link [index, distance, x of article].
    [k_ee,d_ee] = dsearchn(o',punti_robot(:,13)'); % find among all obstacles the one closest to the end-effector
    [d_min,index]=min(dist);
    nob = k(index);
    P_r=segm(:,index*2-1)+x(index)*(segm(:,index*2)-segm(:,index*2-1)); % the Pr position of each link (See article)
    P_o=o(:,k(index)); % position of nearest obstacle
    v_o=V_o(:,k(index)); % speed of nearest obstacle

    %% Calculation r_min and r based on the speed of the nearest obstacle
    %%%%%%%%%%%%%%% 
    if norm(v_o)<v_1
        r_min=r_inf;
    elseif norm(v_o)>v_2
        r_min=r_sup;
    else
        r_min=r_inf+(r_sup-r_inf)/(v_2-v_1)*(norm(v_o)-v_1);
    end
    r=4/3*r_min;
    %%%%%%%%%%%%%%%

    %% Obstacle vectors - control points
    if index>8 
        index=15-1;
        x=[x x];
    end
    J_OA=Jacobian_KMR(Q(:,i),index+1,x(index),P_r); % Jacobian of point Pr considered
    vettore=(P_r-P_o-k_v*v_o*dt); 
    d0=vettore/norm(vettore); 
    d0_vect(:,i) = d0(:,1);
    
    %% null space
    N = eye(10)-pinvd_J*J;
    J_OA_par = J_OA(1:3,:);
    A = J_OA_par*N;
    S = svd(A);
    s_min = S(3);

    if s_min<eps
        lambda_quadro=(1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda_quadro=0;
    end
    pinvd_A = inv(A_J)*A'*inv(A*inv(A_J)*A'+lambda_quadro*eye(3));
    
    %% Don't use null space
    B = J_OA_par; % If I don't want to use null space
    S = svd(B);
    s_min = S(3);

    if s_min<eps
        lambda_quadro=(1-(s_min/eps)^2)*lambda_max^2;
    else
        lambda_quadro=0;
    end
    pinvd_B = inv(A_J)*B'*inv(B*inv(A_J)*B'+lambda_quadro*eye(3));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P_o_ee = o(:,k_ee);
    v_o_ee = V_o(:,k_ee);
    P_r_ee = punti_robot(:,13);

    if  or (d_ee<tolleranza*r_min,d_min<tolleranza*r_min)
        disp 'fail' % If I'm too close, inside the ellipsoid, it's just a warning
    end
    
    distanza_minima(i)=min(d_min,d_ee);
    [a_v, a_h, a_e]=gains(d_min,d_ee,r,r_min);
    e(1:3,1)=X_pl(1:3)-X_ac(1:3,i);
    eul_pl=X_pl(4:6);
    eul_ac=X_ac(4:6,i);
    R_pl=eul2rotm(eul_pl','ZYZ');
    R_ac=eul2rotm(eul_ac','ZYZ');
    e(4:6,1)=(0.5*(cross(R_ac(:,1),R_pl(:,1))+...
        cross(R_ac(:,2),R_pl(:,2))+...
        cross(R_ac(:,3),R_pl(:,3))))/(2*pi);
    d_rep=vettore/norm(vettore);
    dX_rep=a_v*v0_rep*d_rep;
    vettore_ee=(P_r_ee-P_o_ee-k_v*v_o_ee*dt);
    d_rep_ee=vettore_ee/norm(vettore_ee);
    dX_rep_ee=a_e*v0_rep*d_rep_ee;
    dQ_rep = pinvd_J_par*dX_rep_ee;

    if esempio==2
    %% With null space
    dQ=(pinvd_J*(dX(:,i)+k_e.*e)+dQ_rep ...
        +a_h*pinvd_A*(a_v*v0_rep*d0-J_OA_par*pinvd_J*dX(:,i)));
    else
    %% Without null space
    dQ=(pinvd_J*(dX(:,i)+k_e.*e)+dQ_rep ...
        +a_h*pinvd_B*a_v*v0_rep*d0);
    end
    
if zona==1
    
    %% Method to have a planned trajectory that respects the constraints
    dQ=zeros(10,1);
    dQ(1)=(dX_nav(1,i)^2+dX_nav(2,i)^2)^(1/2);
    dQ(3)=dX_nav(6,i);

    %% Yutaka Kanayama Method (A Stable Tracking Control Method for an Autonomous Mobile Robot)
    Arot = [ cos(X_ac_nav(6,i))   sin(X_ac_nav(6,i))  0;
            -sin(X_ac_nav(6,i))   cos(X_ac_nav(6,i))  0;
                          0                0  1];
    eee = [X(1:2,i)-X_ac(1:2,i);X_nav(6,i)-X_ac_nav(6,i)];
    eee = Arot*eee;
    dQ(1)=dQ(1)*cos(eee(3))+K1*eee(1);
    dQ(3)=dQ(3)+dQ(1)*K2*eee(2)+K3*sin(eee(3));

    %% Rotation only method
    yaw = Q(3,i);
    local_versor = [1;0;0];
    vector_n_global = d0;
    vector_n_local = [ vector_n_global(1)*cos(yaw) + vector_n_global(2)*sin(yaw);
                      -vector_n_global(1)*sin(yaw) + vector_n_global(2)*cos(yaw);
                      0];
    segno=cross(local_versor,vector_n_local);
    sin_angle = norm(cross(local_versor,vector_n_local))/norm(vector_n_local)*norm(local_versor); % Angolo tra vettore d0 locale e vettore [1 0 0] dell'asse x locale
    angle_rif = -sign(segno(3))*asind(sin_angle);

    P_r_loc = P_r - X_ac_nav(1:3,i);
    px =  P_r_loc(1)*cos(yaw) + P_r_loc(2)*sin(yaw);
    py = -P_r_loc(1)*sin(yaw) + P_r_loc(2)*cos(yaw); % Pr nel sistema locale
    dQ_rep2 = dQ*0;
    dQ_rep2(3) = a_v*v0_rep/(py*cosd(angle_rif)-px*sind(angle_rif));
    
    dQ=dQ+dQ_rep2;
end

    for j=1:3 % speed limit
        if abs(dQ(j))>dQ_max(j)
            dQ(j)=dQ_max(j)*sign(dQ(j));
        end 
    end
    for j=4:10 % speed limit
        if abs(dQ(j))>dQ_max_g
            dQ(j)=dQ_max_g*sign(dQ(j));
        end 
    end

    dQ(2)=0;
    for j=1:10 % block some joints
        if A_J_diag(j)==0 || A_J_diag(j)>10^15 
            dQ(j)=0;
        end
    end

    Q(:,i+1)=Q(:,i)+dt*dQ;

    dX_ac_nav(:,i+1) = Jacobian_KMR(Q(:,i+1),1,1)*dQ;
    X_ac_nav(:,i+1) = X_ac_nav(:,i) + dt*dX_ac_nav(:,i+1);

    i=i+1;
    [X_app, ~, ~]=kin_dir_UR5_Link(Q(:,i));
    i=i-1;

    X_ac(:,i+1) = X_app;
    dX_ac(:,i+1) = Jacobian_KMR(Q(:,i+1),9,1)*dQ;
    
    dQ_vect(:,i+1)=dQ;

%% To graph the ellipsoids of the mobile robot+manipulator
    fig50 = figure(50);
    fig50.WindowState = 'maximized';
    plot_sfere(O(:,:,i), 0.05); hold on
    plot3(segm(1,1:16),segm(2,1:16),segm(3,1:16),'-ok'); 
    plot3(segm(1,17:end),segm(2,17:end),segm(3,17:end)*0,'-ok');
    plot3(segm(1,17:end),segm(2,17:end),segm(3,17:end)*0+h_mir,'-ok');
    
    corr2 = reshape(corr(1,1:end),[1,2,size(corr,2)/2]);
    segm2 = reshape(segm(:,1:end),[3,2,size(segm,2)/2]);
    for l=1:size(corr2,3)-5
        segPlot(segm2(:,1,l),segm2(:,2,l),r_min,'#0072BD');
    end

    drawCircle(3.0,[5;0;0],[0;0;1],'K',1)
    drawCircle(1.5,[5;0;0],[0;0;1],'K',1)
    grid on
    if zona == 1
        for z = 0.0:0.001:2
            d=plot3(segm(1,17:end),segm(2,17:end),segm(3,17:end)*0+z,'-','Color','#0072BD');
            d.Color(4)=0.10;
        end
    else
        for z = 0.0:0.001:h_mir
            d=plot3(segm(1,17:end),segm(2,17:end),segm(3,17:end)*0+z,'-','Color','#0072BD');
            d.Color(4)=0.10;
        end
    end
%     plot_MIR(Q(:,i),r_min);

    P=X_i(1:3,1);
    eul=X_i(4:6,1);
    R= eul2rotm(eul','ZYZ');
    quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.2,'r')
    quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.2,'g')
    quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),0.2,'b')
    P=X_f(1:3,1);
    eul=X_f(4:6,1);
    R= eul2rotm(eul','ZYZ');
    quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.2,'r')
    quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.2,'g')
    quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),0.2,'b')
    plot3(X_ac(1,1:i),X_ac(2,1:i),X_ac(3,1:i),'-k','linewidth',1);
    plot3(X_ac_nav(1,1:i),X_ac_nav(2,1:i),X_ac_nav(3,1:i),'-k','linewidth',1);
    plot3(X(1,:),X(2,:),X(3,:),'--k','linewidth',1);
    axis equal
    xlim ([min(X_i(1),X_f(1))-1.5 max(X_i(1),X_f(1))+1.5])
    ylim ([min(X_i(2),X_f(2))-1.5 max(X_i(2),X_f(2))+1.5])
    zlim ([0.0 2.0])
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(60,20);
    if esempio==3
        view(30,30); % for Es. 3
    end
    plot3(X_ac(1,i),X_ac(2,i),X_ac(3,i),'*b');
    plot3(X_pl(1),X_pl(2),X_pl(3),'ob');
    hold off
    frame2 = getframe(gcf);
%     writeVideo(vid2,frame2)

%     Frame_i = Frame_i + 1;
%     if i==1
%         saveas(gcf,append('.\Immagini\','Schema',num2str(0),'.png'));
%         print(gcf,append('.\Immagini\','Schema',num2str(0),'.eps'),'-depsc');
%     end
%     if Frame_i>=Frame_count
%         saveas(gcf,append('.\Immagini\','Schema',num2str(i),'.png'));
%         print(gcf,append('.\Immagini\','Schema',num2str(i),'.eps'),'-depsc');
%         Frame_i = 0;
%     end

end
close(vid2);
end
    