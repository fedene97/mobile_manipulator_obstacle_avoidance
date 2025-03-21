function segPlot(A,B,r,color)

n = 10;

l = [A,B];

w = (B-A)./norm(B-A);

u = cross(w,[1 0 0]');
u = u./norm(u);

if isnan(u(1))
    u = cross(w,[0 1 0]');
    u = u./norm(u);
end
if isnan(u(1))
    u = cross(w,[0 0 1]');
    u = u./norm(u);
end

v = cross(w,u);

R = [u v w];

line(l(1,:),l(2,:),l(3,:));

ang = linspace(0,2*pi,n);

[a,b] = meshgrid(ang,ang/4);

Xs = r.*sin(b).*cos(a);
Ys = r.*sin(b).*sin(a);
Zs = r.*cos(b);

xs = reshape(Xs,[1,n*n]);
ys = reshape(Ys,[1,n*n]);
zs = reshape(Zs,[1,n*n]);
s = [xs; ys; zs];

sA = R*([1 0 0; 0 -1 0; 0 0 -1]*s)+A;
sB = R*(s)+B;

XsA = reshape(sA(1,:),[n n]);
YsA = reshape(sA(2,:),[n n]);
ZsA = reshape(sA(3,:),[n n]);

XsB = reshape(sB(1,:),[n n]);
YsB = reshape(sB(2,:),[n n]);
ZsB = reshape(sB(3,:),[n n]);

[a,z] = meshgrid(ang,linspace(0,norm(A-B),n));

Xc = r.*cos(a);
Yc = r.*sin(a);
Zc = z;

xc = reshape(Xc,[1,n*n]);
yc = reshape(Yc,[1,n*n]);
zc = reshape(Zc,[1,n*n]);

cyl = R*[xc; yc; zc]+A;

Xc = reshape(cyl(1,:),[n n]);
Yc = reshape(cyl(2,:),[n n]);
Zc = reshape(cyl(3,:),[n n]);

hold on
surface(XsA,YsA,ZsA,'facealpha',0.1,'edgealpha',0.0,'FaceColor',color)
surface(XsB,YsB,ZsB,'facealpha',0.1,'edgealpha',0.0,'FaceColor',color)
surface(Xc,Yc,Zc,'facealpha',0.1,'edgealpha',0.0,'FaceColor',color)



