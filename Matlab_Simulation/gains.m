function     [a_v, a_h, a_e]=gains(d,d_ee,r,r_min)

d1=r;
d2=(r_min+r)/2;
d3=r_min;

if d>=d1
    a_v=0;
elseif d<=d3
    a_v=1;
else
    a_v=((d-d1)/(d3-d1))^2;
end


if d_ee<=d2
    a_e=1;
else
    if d_ee>=d1
        a_e=0;
    else
        a_e=0.5*(1+cos(pi*(d_ee-d2)/(d1-d2)));
    end
end

if d_ee<=d2
a_h=0;
else
if d<=d2
    a_h=1;
else
    if d>=d1
        a_h=0;
    else
        a_h=0.5*(1+cos(pi*(d-d2)/(d1-d2)));
    end
end
end

end




