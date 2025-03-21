function J = Jacobian_KMR(Q,index,x,P_r)
global X_ac i alphapunta tool X_ac_nav distancex h_mir r_mir l_mir d_mir

xb  = (Q(1));
yb  = (Q(2));
yaw = (Q(3));
qp = yaw;
q1  = (Q(4))+yaw;
q2  = (Q(5));
q3  = (Q(6));
q4  = (Q(7));
q5  = (Q(8));
q6  = (Q(9));
q7  = (Q(10));

[a_2,a_3,d_1,d_4,d_5,d_6,s_1,s_3]=Misure_simulazione();

Q(2) = 0;
Q(10) = 0;

%% Rectangle with point around the mobile robot, 5 points
nav = [-l_mir/2 -l_mir/2   l_mir/2  (d_mir/2)*tand(alphapunta)+(l_mir/2)    l_mir/2;
       -d_mir/2  d_mir/2   d_mir/2                             0   -d_mir/2;
          0    0     0                             0      0];

px = nav(1,4);
py = nav(2,4);
pz = nav(3,4);

J_Mir_mp = ...
            [cos(yaw), 0, 0;
             sin(yaw), 0, 0;
                    0, 0, 0;
                    0, 0, 0;
                    0, 0, 0;
                    0, 0, 1];

J_Mir_mod = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw);
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw);
                    0, 0,                           0;
                    0, 0,                           0;
                    0, 0,                           0;
                    0, 0,                           1];

J_Mir = J_Mir_mod;

%% Choosing the point at which to calculate the Jacobian
switch (index)
    case 1 % MIR under

        J =...
            [ cos(yaw),   0, 0, 0, 0, 0, 0, 0, 0, 0;
              sin(yaw),   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 1, 0, 0, 0, 0, 0, 0, 0];

    case 2 % MIR over/ UR under

        J =...
            [ cos(yaw),   0, 0, 0, 0, 0, 0, 0, 0, 0;
              sin(yaw),   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0,   0, 1, 0, 0, 0, 0, 0, 0, 0];

    case 3 % UR joint 1
        J=...
        [   -distancex*cos(qp)-s_1*x*cos(q1), 0, 0, 0, 0, 0;...
            -distancex*sin(qp)-s_1*x*sin(q1), 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0;...
            0, 0, 0, 0, 0, 0;...
            1, 0, 0, 0, 0, 0];

        J=[J_Mir,J,zeros(6,1)];

    case 4
        J=...
        [   -distancex*cos(qp) - s_1*cos(q1) - a_2*x*cos(q2)*sin(q1), -a_2*x*cos(q1)*sin(q2), 0, 0, 0, 0;...
            -distancex*sin(qp) + a_2*x*cos(q1)*cos(q2) - s_1*sin(q1), -a_2*x*sin(q1)*sin(q2), 0, 0, 0, 0;...
            0,         -a_2*x*cos(q2), 0, 0, 0, 0;...
            0,               -sin(q1), 0, 0, 0, 0;...
            0,                cos(q1), 0, 0, 0, 0;...
            1,                      0, 0, 0, 0, 0];
        
        J=[J_Mir,J,zeros(6,1)];
        
    case 5
        J=...
        [ -distancex*cos(qp) + s_3*x*cos(q1) - s_1*cos(q1) - a_2*cos(q2)*sin(q1), -a_2*cos(q1)*sin(q2),        0, 0, 0, 0;...
          -distancex*sin(qp) + s_3*x*sin(q1) - s_1*sin(q1) + a_2*cos(q1)*cos(q2), -a_2*sin(q1)*sin(q2),        0, 0, 0, 0;...
            0,         -a_2*cos(q2),        0, 0, 0, 0;...
            0,             -sin(q1), -sin(q1), 0, 0, 0;...
            0,              cos(q1),  cos(q1), 0, 0, 0;...
            1,                    0,        0, 0, 0, 0];
        
        J=[J_Mir,J,zeros(6,1)];

    case 6
        J=...
        [ -distancex*cos(qp) + s_3*cos(q1) - s_1*cos(q1) - a_2*cos(q2)*sin(q1) - a_3*x*cos(q2 + q3)*sin(q1), -cos(q1)*(a_2*sin(q2) + a_3*x*sin(q2 + q3)), -a_3*x*sin(q2 + q3)*cos(q1), 0, 0, 0;...
          -distancex*sin(qp) + s_3*sin(q1) - s_1*sin(q1) + a_2*cos(q1)*cos(q2) + a_3*x*cos(q2 + q3)*cos(q1), -sin(q1)*(a_2*sin(q2) + a_3*x*sin(q2 + q3)), -a_3*x*sin(q2 + q3)*sin(q1), 0, 0, 0;...
            0,          - a_2*cos(q2) - a_3*x*cos(q2 + q3),         -a_3*x*cos(q2 + q3), 0, 0, 0;...
            0,                                    -sin(q1),                    -sin(q1), 0, 0, 0;...
            0,                                     cos(q1),                     cos(q1), 0, 0, 0;...
            1,                                           0,                           0, 0, 0, 0];
        
        J=[J_Mir,J,zeros(6,1)];
        
    case 7
        J=...
        [ -distancex*cos(qp) + s_3*cos(q1) - s_1*cos(q1) - d_4*x*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2)), -a_3*sin(q2 + q3)*cos(q1),        0, 0, 0;...
          -distancex*sin(qp) + s_3*sin(q1) - s_1*sin(q1) - d_4*x*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2)), -a_3*sin(q2 + q3)*sin(q1),        0, 0, 0;...
            0,          - a_3*cos(q2 + q3) - a_2*cos(q2),         -a_3*cos(q2 + q3),        0, 0, 0;...
            0,                                  -sin(q1),                  -sin(q1), -sin(q1), 0, 0;...
            0,                                   cos(q1),                   cos(q1),  cos(q1), 0, 0;...
            1,                                         0,                         0,        0, 0, 0];
        
        J=[J_Mir,J,zeros(6,1)];

    case 8
        J=...
        [ -distancex*cos(qp) + s_3*cos(q1) - s_1*cos(q1) - d_4*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1) + d_5*x*sin(q2 + q3 + q4)*sin(q1), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*x*cos(q2 + q3 + q4)), -cos(q1)*(a_3*sin(q2 + q3) + d_5*x*cos(q2 + q3 + q4)), -d_5*x*cos(q2 + q3 + q4)*cos(q1),                          0, 0;...
          -distancex*sin(qp) + s_3*sin(q1) - s_1*sin(q1) - d_4*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2) - d_5*x*sin(q2 + q3 + q4)*cos(q1), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*x*cos(q2 + q3 + q4)), -sin(q1)*(a_3*sin(q2 + q3) + d_5*x*cos(q2 + q3 + q4)), -d_5*x*cos(q2 + q3 + q4)*sin(q1),                          0, 0;...
            0,            d_5*x*sin(q2 + q3 + q4) - a_2*cos(q2) - a_3*cos(q2 + q3),            d_5*x*sin(q2 + q3 + q4) - a_3*cos(q2 + q3),          d_5*x*sin(q2 + q3 + q4),                          0, 0;...
            0,                                                            -sin(q1),                                              -sin(q1),                         -sin(q1), -sin(q2 + q3 + q4)*cos(q1), 0;...
            0,                                                             cos(q1),                                               cos(q1),                          cos(q1), -sin(q2 + q3 + q4)*sin(q1), 0;...
            1,                                                                   0,                                                     0,                                0,         -cos(q2 + q3 + q4), 0];
        
        J=[J_Mir,J,zeros(6,1)];

    case 9
      J =...
[ -distancex*cos(qp) +                                                           s_3*cos(q1) - s_1*cos(q1) - d_4*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1) + d_5*sin(q2 + q3 + q4)*sin(q1) - d_6*x*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                         d_6*x*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)),                                                   0;...
  -distancex*sin(qp) +                                                           s_3*sin(q1) - s_1*sin(q1) - d_4*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2) - d_5*sin(q2 + q3 + q4)*cos(q1) - d_6*x*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                     d_6*x*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - d_6*x*cos(q1)*sin(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,            d_5*sin(q2 + q3 + q4) - a_2*cos(q2) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - d_6*x*cos(q2 + q3 + q4)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                                    -d_6*x*sin(q2 + q3 + q4)*cos(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,                                                                                            -sin(q1),                                                                              -sin(q1),                                                           -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1)*sin(q5) - cos(q5)*sin(q1);...
                                                                                                                                                                                                                                                                      0,                                                                                             cos(q1),                                                                               cos(q1),                                                            cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*sin(q1), cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5);...
(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5))^2 + (sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))^2 + (cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))^2,                                                                                                   0,                                                                                     0,                                                                  0, (cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1))*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q6)*(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - sin(q6)*(sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)),                          -sin(q2 + q3 + q4)*sin(q5)];
 
      J=[J_Mir,J,zeros(6,1)];





    case 10
        px = nav(1,1);
        py = nav(2,1);
        pz = nav(3,1);

        J = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
                    0, 0,                           0, 0, 0, 0, 0, 0, 0, 0];
        J=[J; zeros(3,10)];
        J(6,1) = 0;
        J(6,3) = 1;

    case 11
        px = nav(1,2);
        py = nav(2,2);
        pz = nav(3,2);

        J = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
                    0, 0,                           0, 0, 0, 0, 0, 0, 0, 0];
        J=[J; zeros(3,10)];
        J(6,1) = 0;
        J(6,3) = 1;

    case 12
        px = nav(1,3);
        py = nav(2,3);
        pz = nav(3,3);

        J = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
                    0, 0,                           0, 0, 0, 0, 0, 0, 0, 0];
        J=[J; zeros(3,10)];
        J(6,1) = 0;
        J(6,3) = 1;

    case 13
        px = nav(1,4);
        py = nav(2,4);
        pz = nav(3,4);

        J = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
                    0, 0,                           0, 0, 0, 0, 0, 0, 0, 0];
        J=[J; zeros(3,10)];
        J(6,1) = 0;
        J(6,3) = 1;

    case 14
        px = nav(1,5);
        py = nav(2,5);
        pz = nav(3,5);

        J = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
                    0, 0,                           0, 0, 0, 0, 0, 0, 0, 0];
        J=[J; zeros(3,10)];
        J(6,1) = 0;
        J(6,3) = 1;

    case 15 % Generic case with px, py relative to Pr
        P_r_loc = P_r - X_ac_nav(1:3,i);
        P_r_loc = [ P_r_loc(1)*cos(yaw) + P_r_loc(2)*sin(yaw);
                   -P_r_loc(1)*sin(yaw) + P_r_loc(2)*cos(yaw);
                   0];

        px = P_r_loc(1);
        py = P_r_loc(2);
        pz = 0;

        J = ...
            [cos(yaw), 0, - py*cos(yaw) - px*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
             sin(yaw), 0,   px*cos(yaw) - py*sin(yaw), 0, 0, 0, 0, 0, 0, 0;
                    0, 0,                           0, 0, 0, 0, 0, 0, 0, 0];
        J=[J; zeros(3,10)];
        J(6,1) = 0;
        J(6,3) = 1;

        case 16
      J =...
[ -distancex*cos(qp) +                                                           s_3*cos(q1) - s_1*cos(q1) - d_4*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1) + d_5*sin(q2 + q3 + q4)*sin(q1) - d_6*x*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                         d_6*x*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)),                                                   0;...
  -distancex*sin(qp) +                                                           s_3*sin(q1) - s_1*sin(q1) - d_4*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2) - d_5*sin(q2 + q3 + q4)*cos(q1) - d_6*x*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                     d_6*x*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - d_6*x*cos(q1)*sin(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,            d_5*sin(q2 + q3 + q4) - a_2*cos(q2) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - d_6*x*cos(q2 + q3 + q4)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                                    -d_6*x*sin(q2 + q3 + q4)*cos(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,                                                                                            -sin(q1),                                                                              -sin(q1),                                                           -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1)*sin(q5) - cos(q5)*sin(q1);...
                                                                                                                                                                                                                                                                      0,                                                                                             cos(q1),                                                                               cos(q1),                                                            cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*sin(q1), cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5);...
(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5))^2 + (sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))^2 + (cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))^2,                                                                                                   0,                                                                                     0,                                                                  0, (cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1))*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q6)*(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - sin(q6)*(sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)),                          -sin(q2 + q3 + q4)*sin(q5)];
 
      J=[J_Mir_mod,J,zeros(6,1)];

        case 17
      J =...
[ -distancex*cos(qp) +                                                           s_3*cos(q1) - s_1*cos(q1) - d_4*cos(q1) - a_3*cos(q2 + q3)*sin(q1) - a_2*cos(q2)*sin(q1) + d_5*sin(q2 + q3 + q4)*sin(q1) - d_6*x*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -cos(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                         d_6*x*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)),                                                   0;...
  -distancex*sin(qp) +                                                           s_3*sin(q1) - s_1*sin(q1) - d_4*sin(q1) + a_3*cos(q2 + q3)*cos(q1) + a_2*cos(q1)*cos(q2) - d_5*sin(q2 + q3 + q4)*cos(q1) - d_6*x*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + a_2*sin(q2) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(a_3*sin(q2 + q3) + d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)), -sin(q1)*(d_5*cos(q2 + q3 + q4) + d_6*x*sin(q2 + q3 + q4)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                     d_6*x*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - d_6*x*cos(q1)*sin(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,            d_5*sin(q2 + q3 + q4) - a_2*cos(q2) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - a_3*cos(q2 + q3) - d_6*x*cos(q2 + q3 + q4)*sin(q5),            d_5*sin(q2 + q3 + q4) - d_6*x*cos(q2 + q3 + q4)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                                    -d_6*x*sin(q2 + q3 + q4)*cos(q5),                                                   0;...
                                                                                                                                                                                                                                                                      0,                                                                                            -sin(q1),                                                                              -sin(q1),                                                           -sin(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1)*sin(q5) - cos(q5)*sin(q1);...
                                                                                                                                                                                                                                                                      0,                                                                                             cos(q1),                                                                               cos(q1),                                                            cos(q1),                                                                                                                                                                                                                                                                                                                                                                                                                          -sin(q2 + q3 + q4)*sin(q1), cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5);...
(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5))^2 + (sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))^2 + (cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))^2,                                                                                                   0,                                                                                     0,                                                                  0, (cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1))*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q6)*(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - sin(q6)*(sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*cos(q6))*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)),                          -sin(q2 + q3 + q4)*sin(q5)];

      J=[J_Mir_mp,J,zeros(6,1)];


end

end

