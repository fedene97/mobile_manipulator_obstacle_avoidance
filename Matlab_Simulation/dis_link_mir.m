function [k,d,x]=dis_link_mir(punti_robot_g,o)
% link
link = [];
for g= 2:2:16
    link_app = punti_robot_g(:,g)-punti_robot_g(:,g-1);
    link=[link link_app];
end
for g= 18:2:size(punti_robot_g,2)
    link_app = punti_robot_g(:,g)-punti_robot_g(:,g-1);
    link=[link link_app];
end
int=zeros(size(o,2),size(link,2));
distanza=zeros(size(o,2),size(link,2));
d=zeros(size(link,2),1);
k=zeros(size(link,2),1);
x=zeros(size(link,2),1);
for n=1:size(o,2)
    for i=1:size(link,2)
        d_p=o(:,n)-punti_robot_g(:,i*2-1);
        d_d=o(:,n)-punti_robot_g(:,i*2);
        
        c_a=(dot(d_p,link(:,i)))/(norm(d_p)*norm(link(:,i)));
        c_b=(dot(d_d,-link(:,i)))/(norm(d_d)*norm(link(:,i)));

        if c_a >= 0 && c_b >=0
            distanza(n,i)=norm(cross(link(:,i),d_d))/norm(link(:,i));
            int(n,i)=norm(d_p)*c_a/norm(link(:,i));   
        elseif c_a < 0 && c_b > 0
            distanza(n,i)=norm(d_p);
            int(n,i)=0;
        elseif c_b < 0 && c_a > 0
            distanza(n,i)=norm(d_d);
            int(n,i)=1;
        end
    end
end

for s=1:size(link,2)
    [d(s), k(s)]=min(distanza(:,s));
    x(s)=int(k(s),s);
end


