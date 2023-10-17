function [n_q_q_dot]=n(q,q_dot)

theta1 = q(1);
theta2 = q(2);
d3 = q(3);
theta4 = q(4);

theta1_dot = q_dot(1);
theta2_dot = q_dot(2);
    
    l1 = 0.25;
    l2 = 0.25;
    ml1 = 25;
    ml2 = 25;
    ml3 = 10;
    me = 5;  % Mass of load being carried by the end-effector
    ml4 =5;
    kr1 = 1;
    kr2 = 1;
    kr3 = 50;
    kr4 = 20;
    Fm1 = 0.0001;
    Fm2 = 0.0001;
    Fm3 = 0.01;
    Fm4 = 0.005;

    g0 = [0 0 -9.8]';

%Base to Zero frame 
%d0 =1m 
HB_0 = DH(1, 0, 0 ,0);
H0_1 = DH(0 , 0 , theta1 , 0.5);
H1_2 = DH(0 , 0 , theta2, 0.5);
H2_3 = DH(-d3,0 ,0 ,0); 
H3_4 = DH(0 , 0 , theta4, 0);
HB_1 = (HB_0 * H0_1);
HB_2 = (HB_0*H0_1*H1_2);
HB_3 = (HB_0*H0_1*H1_2*H2_3);
HB_4 = HB_0 * H0_1 * H1_2 * H2_3 * H3_4;

pm1 = [HB_0(1,4) , HB_0(2,4) , HB_0(3,4)];
pm2 = [HB_1(1,4) , HB_1(2,4) , HB_1(3,4)];
pm3 = [HB_2(1,4) , HB_2(2,4) , HB_2(3,4)];
pm4 = [HB_3(1,4) , HB_3(2,4) , HB_3(3,4)];

pcom_l1 = [l1*cos(theta1) , l2*sin(theta1) , 0];
pcom_l2 = [l2*cos(theta1+ theta2) , l2*sin(theta1+theta2) , 0];

pl1 = pm1 + pcom_l1;
pl2 = pm2 + pcom_l2;
pl3 = pm3;
pl4 = pm4;

%Using the formula given, we compute a matrix cijk a 4x4x4 matrix where we
%differentiate every element of B-matrix with joint values q1 , q2,q3,q4.
%After forming a cijk matrix we convert every element to a cij form using
%the given formula
C = [(-55 *sin(theta2)*theta2_dot)/ 8 , (-55*sin(theta2)*(theta1_dot + theta2_dot))/8, 0 , 0;
      (55*sin(theta2)*theta1_dot )/8, 0 ,0 ,0;
      0, 0, 0, 0;
      0, 0, 0, 0];

%finding g(q)
%Mass of motor is 0

%Finding Potential energy of one joint with every link

%Jp1_l1
pl1_p0 = [pl1(1,1)-HB_0(1,4) , pl1(1,2)-HB_0(2,4) , pl1(1,3)-HB_0(3,4)];
z0 = [HB_0(1,3) , HB_0(2,3) , HB_0(3,3)];
Jp1_l1 = cross(z0, pl1_p0); 

%Jp1_l2
pl2_p0 = [pl2(1,1)-HB_0(1,4), pl2(1,2)-HB_0(2,4), pl2(1,3)-HB_0(3,4)];
Jp1_l2 = cross(z0 , pl2_p0);

%Jp1_l3
pl3_p0 = [pl3(1,1)-HB_0(1,4), pl3(1,2)-HB_0(2,4), pl3(1,3)-HB_0(3,4)];
Jp1_l3 = cross(z0,pl3_p0);

%Jp1_l4
pl4_p0 = [pl4(1,1)-HB_0(1,4) , pl4(1,2)-HB_0(2,4) , pl4(1,3)-HB_0(3,4)];
Jp1_l4 = cross(z0, pl4_p0);

g1 = (ml1 * (g0)' * Jp1_l1') + (ml2 * (g0)' * Jp1_l2') + (ml3*(g0)' * Jp1_l3') + (ml4 * (g0)' * Jp1_l4');

%Jp2_l1
z1  = [HB_1(1,3) , HB_1(2,3) , HB_1(3,3)];
pl1_p1 = [pl1_p0(1,1)-HB_1(1,4) , pl1(1,2)-HB_1(2,4) , pl1(1,3)-HB_1(3,4)];
Jp2_l1 = cross(z1,pl1_p1);

%Jp2_l2
pl2_p1 = [pl2(1,1)-HB_1(1,4) , pl2(1,2)-HB_1(2,4) , pl2(1,3)-HB_1(3,4)];
Jp2_l2 = cross(z1, pl2_p1);

%Jp2_l3
pl3_p1 = [pl3(1,1)-HB_1(1,4) , pl3(1,2)-HB_1(2,4) , pl3(1,3)-HB_1(3,4)];
Jp2_l3 = cross(z1,pl3_p1);

%Jp2_l4
pl4_p1 = [pl4(1,1)-HB_1(1,4) , pl4(1,2)-HB_1(2,4) , pl4(1,3)-HB_1(3,4)];
Jp2_l4 = cross(z1,pl4_p1);

g2 = -(ml1 * (g0)' * Jp2_l1') + (ml2 * (g0)' * Jp2_l2') + (ml3*(g0)' * Jp2_l3') + (ml4 * (g0)' * Jp2_l4');

%Jp3_l1
z2 = [HB_2(1,3) , HB_2(2,3) , HB_2(3,3)];
Jp3_l1 = z2;
%Jp3_l2
Jp3_l2 = z2;
%Jp3_l3
Jp3_l3 = z2;
%Jp3_l4
Jp3_l4 = z2;

g3 = -(ml1 * (g0)' * Jp3_l1') + (ml2 * (g0)' * Jp3_l2') + (ml3*(g0)' * Jp3_l3') + (ml4 * (g0)' * Jp3_l4');

%Jp4_l1
z3 = [HB_3(1,3) , HB_3(2,3) , HB_3(3,3)];
pl1_p3 = [pl1(1,1)-HB_3(1,4) , pl1(1,2)-HB_3(2,4) , pl1(1,3)-HB_3(3,4)];
Jp4_l1 = cross(z3,pl1_p3);

%Jp4_l2
pl2_p3 = [pl2(1,1)-HB_3(1,4) , pl2(1,2)-HB_3(2,4) , pl2(1,3)-HB_3(3,4)];
Jp4_l2 = cross(z3,pl2_p3);

%Jp4_l3
pl3_p3 = [pl3_p1(1,1)-HB_3(1,4) , pl3(1,2)-HB_3(2,4) , pl3(1,3)-HB_3(3,4)];
Jp4_l3 = cross(z3,pl3_p3);

%Jp4_l4
pl4_p3 = [pl4(1,1)-HB_3(1,4) , pl4(1,2)-HB_3(2,4) , pl4(1,3)-HB_3(3,4)];
Jp4_l4 = cross(z3 , pl4_p3);

g4 = -(ml1 * (g0)' * Jp4_l1') + (ml2 * (g0)' * Jp4_l2') + (ml3*(g0)' * Jp4_l3') + (ml4 * (g0)' * Jp4_l4');

G = [g1 ; g2 ; g3 ; g4];

F = [kr1^2*Fm1 0 0 0;
     0 kr2^2*Fm2 0 0;
     0 0 kr3^2*Fm3 0;
     0 0 0 kr4^2*Fm4];

n_q_q_dot = C*q_dot + F *q_dot + G;
end

function  [A] = DH(d , alpha , theta, a)
A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0 sin(alpha) cos(alpha) d ;
     0  0  0 1 ];  
end

