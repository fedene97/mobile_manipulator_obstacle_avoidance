function plot_UR5_link(Q,r)
global UR5

[X, punti_robot]=kin_dir_UR5_Link(Q);
punti_robot_g(:,1)=punti_robot(:,2);%A
punti_robot_g(:,2)=punti_robot(:,3);%B
punti_robot_g(:,3)=punti_robot(:,6);%C
punti_robot_g(:,4)=punti_robot(:,7);%C1
punti_robot_g(:,5)=punti_robot(:,10);%D
punti_robot_g(:,6)=punti_robot(:,11);%E
punti_robot_g(:,7)=punti_robot(:,12);%F
punti_robot_g(:,8)=punti_robot(:,13);%G==EE

plot3([punti_robot_g(1,:)],[punti_robot_g(2,:)],[punti_robot_g(3,:)],'-ok')
hold on

for i=1:size(punti_robot_g,2)
    
    centro=punti_robot_g(:,i);
    [x,y,z] = sphere(10);
    surf(r*x+centro(1),r*y+centro(2),r*z+centro(3),...
        'FaceColor','none','EdgeColor',[0.8 0.9 1])
end

for j=1:7
    prox=punti_robot_g(1:3,j);
    dist=punti_robot_g(1:3,j+1);
    l=norm(dist-prox);
    [x,y,z] = cylinder(r);
    z=l*z;
    if cross([0;0;1], (dist-prox))==0
    else
        u=cross([0;0;1], (dist-prox))/norm(cross([0;0;1], (dist-prox)));
        sin_a=norm(cross([0;0;1], (dist-prox)))/norm(dist-prox);
        if dot([0;0;1],(dist-prox))==0
            cos_a=0;
        else
            cos_a=dot([0;0;1],(dist-prox))/norm(dist-prox);
        end
        a=atan2(sin_a,cos_a);
        axang = [u' a];
        rotm = axang2rotm(axang);
        point_inf=[x(1,:);y(1,:);z(1,:)];
        point_sup=[x(2,:);y(2,:);z(2,:)];
        point_inf_rot=rotm*point_inf;
        point_sup_rot=rotm*point_sup;
        x=[point_inf_rot(1,:);point_sup_rot(1,:)];
        y=[point_inf_rot(2,:);point_sup_rot(2,:)];
        z=[point_inf_rot(3,:);point_sup_rot(3,:)];
        surf(x+prox(1),y+prox(2),z+prox(3),...
            'FaceColor','none','EdgeColor',[0.8 0.9 1])
    end
    
end

P=zeros(3,1);
eul=zeros(3,1);
R= eul2rotm(eul','ZYZ');
quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.2,'r')
quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.2,'g')
quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),0.2,'b')

P=punti_robot_g(1:3,8);
eul=punti_robot_g(4:6,8);
R= eul2rotm(eul','ZYZ');
quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),0.2,'r')
quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),0.2,'g')
quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),0.2,'b')


% show(UR5,Q(4:9)', 'Frames','off','position',p0);

end
