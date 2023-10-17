function [B_q_dot_dot]=B(q,y)
    
    d0 = 1;
    a1 =0.5;
    a2 = 0.5;
    l1 = 0.25;
    l2 = 0.25;
    ml1 = 25;
    ml2 = 25;
    ml3 = 10;
    me = 5;  % Mass of load being carried by the end-effector
    ml4 = 5;
    Il1 = 5;
    Il2 = 5;
    Il4 = 1;
    kr1 = 1;
    kr2 = 1;
    kr3 = 50;
    kr4 = 20;
    Im1 = 0.0001;
    Im2 = 0.0001;
    Im3 = 0.01;
    Im4 = 0.005;
    Fm1 = 0.0001;
    Fm2 = 0.0001;
    Fm3 = 0.01;
    Fm4 = 0.005;
    theta1 = q(1);
    theta2 = q(2);
    d3 = q(3);
    theta4 = q(4);


%Base to Zero frame 
%d0 =1m 
HB_0 = DH(1, 0, 0 ,0);
H0_1 = DH(0 , 0 , theta1 , 0.5);
H1_2 = DH(0 , 0 , theta2, 0.5);
H2_3 = DH(-d3,0 ,0 ,0); 
H3_4 = DH(0 , 0 , theta4, 0);
%Frame1 wrt Base frame
HB_1 = (HB_0 * H0_1);
%Frame2 wrt Base frame
HB_2 = (HB_0*H0_1*H1_2);
%Frame3 wrt Base
HB_3 = (HB_0*H0_1*H1_2*H2_3);
%Direct Kinematics (Final Homogenous Matrix) Base to End Effector
HB_4 = HB_0 * H0_1 * H1_2 * H2_3 * H3_4;

%Position of each joint (translation part of the Homogenous matrix) wrt base is position of the motor
%There are 4 motors
pm1 = [HB_0(1,4) , HB_0(2,4) , HB_0(3,4)];
pm2 = [HB_1(1,4) , HB_1(2,4) , HB_1(3,4)];
pm3 = [HB_2(1,4) , HB_2(2,4) , HB_2(3,4)];
pm4 = [HB_3(1,4) , HB_3(2,4) , HB_3(3,4)];

%Calculating the position of the center of mass for each link
pcom_l1 = [l1*cos(theta1) , l2*sin(theta1) , 0];
pcom_l2 = [l2*cos(theta1+ theta2) , l2*sin(theta1+theta2) , 0];

%For the third and fourth link, since the question does not state the val. of l3 and l4 we
%Consider the position of the center of mass to be the pos of the joint (motor)
%Thus the position of the link is considered as the position of motor

%Calculating the position of each link
pl1 = pm1 + pcom_l1;
pl2 = pm2 + pcom_l2;
pl3 = pm3;
pl4 = pm4;

%Jacobian for links % i - link no. , j - joint no.
%Jacobian - Linear velocity- Jp


%Jp_l1
pl1_p0 = [pl1(1,1)-HB_0(1,4) , pl1(1,2)-HB_0(2,4) , pl1(1,3)-HB_0(3,4)];
z0 = [HB_0(1,3) , HB_0(2,3) , HB_0(3,3)];
Jp_1 = cross(z0, pl1_p0);

Jp_l1 = [Jp_1(1,1) 0 0 0; 
        Jp_1(1,2) 0 0 0 ; 
        Jp_1(1,3) 0 0 0 ; 
        0 0 0 0];
%Jp_l2 
pl2_p1 = [pl2(1,1)-HB_1(1,4) , pl2(1,2)-HB_1(2,4) , pl2(1,3)-HB_1(3,4)];
z1  = [HB_1(1,3) , HB_1(2,3) , HB_1(3,3)];
Jp_2 = cross(z1, pl2_p1);
    %first column will have Jp1_l2 %since considering vel. of link1 when
    %computing link2
pl2_p0 = [pl2(1,1)-HB_0(1,4), pl2(1,2)-HB_0(2,4), pl2(1,3)-HB_0(3,4)];
Jp1_l2 = cross(z0 , pl2_p0);
Jp_l2 = [Jp1_l2(1,1) Jp_2(1,1) 0 0;
         Jp1_l2(1,2) Jp_2(1,2) 0 0;
         Jp1_l2(1,3) Jp_2(1,3) 0 0;
         0 0 0 0];

%Jp_l3 - prismatic joint 
z2 = [HB_2(1,3) , HB_2(2,3) , HB_2(3,3)];
pl3_p0 = [pl3(1,1)-HB_0(1,4), pl3(1,2)-HB_0(2,4), pl3(1,3)-HB_0(3,4)];
pl3_p1 = [pl3(1,1)-HB_1(1,4), pl3(1,2)-HB_1(2,4), pl3(1,3)-HB_1(3,4)];
Jp_3 = z2;
Jp1_l3 = cross(z0,pl3_p0);
Jp2_l3 = cross(z1,pl3_p1);
Jp_l3 = [Jp1_l3(1,1) Jp2_l3(1,1) Jp_3(1,1) 0;
         Jp1_l3(1,2) Jp2_l3(1,2) Jp_3(1,2) 0;
         Jp1_l3(1,3) Jp2_l3(1,3) Jp_3(1,3) 0;
         0 0 0 0];

%Jp_l4 
pl4_p3 = [pl4(1,1)-HB_3(1,4) , pl4(1,2)-HB_3(2,4) , pl4(1,3)-HB_3(3,4)];
pl4_p0 = [pl4(1,1)-HB_0(1,4) , pl4(1,2)-HB_0(2,4) , pl4(1,3)-HB_0(3,4)];
pl4_p1 = [pl4(1,1)-HB_1(1,4) , pl4(1,2)-HB_1(2,4) , pl4(1,3)-HB_1(3,4)];
pl4_p2 = [pl4(1,1)-HB_2(1,4) , pl4(1,2)-HB_2(2,4) , pl4(1,3)-HB_2(3,4)];

z3 = [HB_3(1,3), HB_3(2,3) , HB_4(3,3)];
Jp_4 = cross(z3 , pl4_p3);
Jp1_l4 = cross(z0, pl4_p0);
Jp2_l4 = cross(z1, pl4_p1);
Jp3_l4 = z2; % Joint 3 wrt link4
Jp_l4 = [Jp1_l4(1,1) Jp2_l4(1,1) Jp3_l4(1,1) Jp_4(1,1);
         Jp1_l4(1,2) Jp2_l4(1,2) Jp3_l4(1,2) Jp_4(1,2);
         Jp1_l4(1,3) Jp2_l4(1,3) Jp3_l4(1,3) Jp_4(1,3);
         0 0 0 0];

%Jacobian -links- Angular velocity - Jo

%Jo_l1
%using prev. computed z0
Jo_1 = z0;  %z0 = [HB_0(1,3) , HB_0(2,3) , HB_0(3,3)];
Jo_l1 = [Jo_1(1,1) 0 0 0;
         Jo_1(1,2) 0 0 0;
         Jo_1(1,3) 0 0 0
         0 0 0 0];

%Jo_l2
%using prev. computed z1
Jo_2 = z1; %z1  = [HB_1(1,3) , HB_1(2,3) , HB_1(3,3)];
Jo_l2 = [Jo_1(1,1) Jo_2(1,1) 0 0;
         Jo_1(1,2) Jo_2(1,2) 0 0;
         Jo_1(1,3) Jo_2(1,3) 0 0
         0 0 0 0];

%Jo_l3 - prismatic joint - angular vel. is 0
Jo_l3 = [Jo_1(1,1) Jo_2(1,1) 0 0;
         Jo_1(1,2) Jo_2(1,2) 0 0;
         Jo_1(1,3) Jo_2(1,3) 0 0
         0 0 0 0];

%Jo_l4 
%using prev. computed z3
Jo_4 = z3; %z3 = [HB_3(1,3), HB_3(2,3) , HB_4(3,3)];
Jo_l4 = [Jo_1(1,1) Jo_2(1,1) 0 Jo_4(1,1);
         Jo_1(1,2) Jo_2(1,2) 0 Jo_4(1,2);
         Jo_1(1,3) Jo_2(1,3) 0 Jo_4(1,3)
         0 0 0 0];

%Jacobian for motors % Will be 0 because considering mass of motor is 0.
%Jacobian - linear velocity - Jp

%Jp_m1 
pm1_p0 = [pm1(1,1)-HB_0(1,4) , pm1(1,2)-HB_0(2,4) , pm1(1,3)-HB_0(3,4)];
Jp_m1_1 = cross(z0, pm1_p0);
Jp_m1 = [Jp_m1_1(1,1) 0 0 0;
         Jp_m1_1(1,2) 0 0 0;
         Jp_m1_1(1,3) 0 0 0;
         0 0 0 0];

%Jp_m2
pm2_p1 = [pm2(1,1)-HB_1(1,4) ,pm2(1,2)-HB_1(2,4) , pm2(1,3)-HB_1(3,4)];
Jp_m2_1 = cross(z1,pm2_p1);
Jp_m2 = [Jp_m1_1(1,1) Jp_m2_1(1,1) 0 0;
         Jp_m1_1(1,2) Jp_m2_1(1,2) 0 0;
         Jp_m1_1(1,3) Jp_m2_1(1,3) 0 0;
         0 0 0 0];

%Jp_m3 - prismatic
Jp_m3_1 = z2;
Jp_m3 = [Jp_m1_1(1,1) Jp_m2_1(1,1) Jp_m3_1(1,1) 0;
         Jp_m1_1(1,2) Jp_m2_1(1,2) Jp_m3_1(1,2) 0;
         Jp_m1_1(1,3) Jp_m2_1(1,3) Jp_m3_1(1,3) 0;
         0 0 0 0];

%Jp_m4
pm4_p3 = [pm4(1,1)-HB_3(1,4) , pm4(1,2)-HB_3(2,4) , pm4(1,3)-HB_3(3,4)];
Jp_m4_1 = cross(z3,pm4_p3);
Jp_m4 = [Jp_m1_1(1,1) Jp_m2_1(1,1) Jp_m3_1(1,1) Jp_m4_1(1,1);
         Jp_m1_1(1,2) Jp_m2_1(1,2) Jp_m3_1(1,2) Jp_m4_1(1,2);
         Jp_m1_1(1,3) Jp_m2_1(1,3) Jp_m3_1(1,3) Jp_m4_1(1,3);
         0 0 0 0];

%Jacobian -Motor- angular velocity -Jo
%Jo_m1
%zmi - zm1 - which means that z axis of first motor: z0 (since 1st motor is at frame0)
Jo_m1_1 = kr1*z0;
Jo_m1 = [Jo_m1_1(1,1) 0 0 0;
         Jo_m1_1(1,2) 0 0 0;
         Jo_m1_1(1,3) 0 0 0;
         0 0 0 0];
%Jo_m2
%For any value j not equal to i , Jo_1
%zmi - zm2 - which means that z axis of second motor: z1 (since 2nd motor is at frame1)
Jo_m2_1 = kr2*z1;
Jo_m2 = [Jo_1(1,1) Jo_m2_1(1,1) 0 0;
         Jo_1(1,2) Jo_m2_1(1,2) 0 0;
         Jo_1(1,3) Jo_m2_1(1,3) 0 0;
         0 0 0 0];

%Jo_m3 - prismatic
%For any value j not equal to i , Jo_1 and Jo_2
%zmi - zm3 - which means that z axis of third motor: z2 (since 3rd motor is at frame2)
Jo_m3_1 = kr3*z2;
Jo_m3 = [Jo_1(1,1) Jo_2(1,1) Jo_m3_1(1,1) 0;
         Jo_1(1,2) Jo_2(1,2) Jo_m3_1(1,2) 0;
         Jo_1(1,3) Jo_2(1,3) Jo_m3_1(1,3) 0;
         0 0 0 0];

%Jo_m4
%For any value j not equal to i , Jo_1 Jo_2 and Jo_3
%zmi - zm4 - which means that z axis of fourth motor: z3 (since 4th motor is at frame3)
Jo_m4_1 = kr4*z3;         
%third column is zeros because Jo_3 is a matrix of 0 (angular velocity for the prismatic joint is 0)
Jo_m4 = [Jo_1(1,1) Jo_2(1,1) 0  Jo_m4_1(1,1);
         Jo_1(1,2) Jo_2(1,2) 0  Jo_m4_1(1,2);
         Jo_1(1,3) Jo_2(1,3) 0  Jo_m4_1(1,3);
         0 0 0 0];

%computing the B matrix
%M4 = mass of load being carried by the end effector = 5    
B_1 = (ml1*(Jp_l1)' * (Jp_l1)) + ((Jo_l1)' * Il1 * (Jo_l1)) + 0 + (Jo_m1)'* Im1 * (Jo_m1);
B_2 = (ml2*(Jp_l2)' * (Jp_l2)) + ((Jo_l2)' * Il2 * (Jo_l2)) + 0 + (Jo_m2)'* Im2 * (Jo_m2);
B_3 = (ml3*(Jp_l3)' * (Jp_l3)) + ((Jo_l3)' * 0 * (Jo_l3)) + 0 + (Jo_m3)'* Im3 * (Jo_m3);
B_4 = (me*(Jp_l4)' * (Jp_l4)) + ((Jo_l4)' * Il4 * (Jo_l4)) + 0 + (Jo_m4)'* Im4 * (Jo_m4);

B_q = B_1 + B_2 + B_3 + B_4;
B_q_dot_dot = B_q * y;

end

function  [A] = DH(d , alpha , theta, a)
A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0 sin(alpha) cos(alpha) d ;
     0  0  0 1 ];  
end

