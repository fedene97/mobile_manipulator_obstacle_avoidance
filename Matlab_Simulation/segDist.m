function [dist,ind] = segDist(A,B,C)

u = (B-A)/norm(B-A);
v = (C-A)/norm(C-A);


d1 = norm(C-A)*sqrt(1-(u'*v)^2);
d2 = norm(C-A);
d3 = norm(C-B);

dist = min([d1 d2 d3]);

if (C-A)'*u<0
   ind = 1;
   dist = norm(C-A);
elseif (C-A)'*u>norm(B-A)
   ind = 0;
   dist = norm(C-B);
else
   ind = 1-((C-A)'*u)/norm(B-A);
   dist = norm(cross(C-A,u));
end

if isnan(ind)
    ind=0;
end