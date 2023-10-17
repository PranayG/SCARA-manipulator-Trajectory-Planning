%main code
clear all;
close all;
clc
Ts=0.001;


p0 = [0 -0.80 0];
p1 = [0 -0.80 0.5];
p2 = [0.5 -0.6 0.5];
p3 = [0.8 0.0 0.5];
p4 = [0.8 0.0 0.0];

t0 = 0.0;
t1 = 0.6;
t2 = 2.0;
t3 = 3.4;
t4 = 4.0;
t = (t0:Ts:t3)';
q0 = [-0.927293431584587;-1.287005790420619;0.700000000000000;2.214299222005206];

%There are 4 segments for the trajectory. Each segment will have separate
%values for tc and q_dot_dot_c

%First Segment p0 to p1
tf_1 = t1; % 0.6 -0 = 0.6 %tf is the time interval you want the manipulator to work for
qf_1 = p1;
qi_1 = p0;
qt_1 = [];

q_dot_dot_c_1 = (4*(qf_1 - qi_1)/(tf_1)^2) * 1.125 ;
tc_1 = tf_1/2 - 0.5* sqrt((((tf_1)^2 * norm(q_dot_dot_c_1))-(4 * norm(qf_1 - qi_1)))/norm(q_dot_dot_c_1));

%sample_1 = 0.6/0.001;

for T = 0:0.001:0.6
    if (T>=0 && tc_1>=T)
        qt_1(end+1, :) = qi_1+ 0.5*q_dot_dot_c_1*T^2;
    elseif(T>tc_1 && T<=(tf_1 - tc_1))
        qt_1(end+1, :)=  qi_1+ q_dot_dot_c_1*tc_1*(T-tc_1/2);
    elseif(tf_1 - tc_1 <T && T<=tf_1)
        qt_1(end+1, :) =qf_1 - 0.5*(q_dot_dot_c_1)*(tf_1 - T)^2;
    end
    
end 

%Considering anticipation time for calculating arclength

%Segment 1 is not affected by anticipation time
s1 = zeros(3401,1);
d1 = 0 ;
for i = 1:600
   d1 = norm(qt_1(i+1,:)-qt_1(i,:)) + d1;
   s1(i+1) = d1;
end
s1(602:3401,1) = s1(601,1);

%Second Segment p1 to p2
tf_2 = t2 - t1; % 2-0.6 = 1.4 %tf is the time interval you want the manipulator to work for
qf_2 = p2;
qi_2 = p1;
qt_2 = [];

q_dot_dot_c_2 = (4*(qf_2 - qi_2)/(tf_2)^2) * 1.125 ;
tc_2 = tf_2/2 - 0.5* sqrt((((tf_2)^2 * norm(q_dot_dot_c_2))-(4 * norm(qf_2 - qi_2)))/norm(q_dot_dot_c_2));


for T = (0:0.001:1.4)
    if (T>=0 && tc_2>=T)
        qt_2(end+1, :) = qi_2+ 0.5*q_dot_dot_c_2*T^2;
    elseif(T>tc_2 && T<=(tf_2 - tc_2))
        qt_2(end+1, :)=  qi_2+ q_dot_dot_c_2*tc_2*(T-tc_2/2);
    elseif (tf_2 - tc_2)<T && (T<=tf_2)
        qt_2(end+1, :) =qf_2 - 0.5*(q_dot_dot_c_2)*(tf_2 - T)^2;
    end
end 

%Considering anticipation time for calculating arclength

%Segment2 anticipation time starts from 0.4 and ends at 1.8
% 0 to 0.4 = Segment2 will be 0
% After 1.8 , segment2 will remain constant

s2 = zeros(3401,1);
d2 = 0 ;
s2_temp = [0];
for i = 1:1399
   d2 = norm(qt_2(i+1,:)-qt_2(i,:)) + d2;
   s2_temp(end+1,:) = d2;
end
s2(401:1800,1) = s2_temp(1:1400,1);
s2(1801:3401,1) = s2_temp(1400,1);

%Third Segment p2 to p3
tf_3 = t3 - t2; % 3.4 - 2 = 1.4 %tf is the time interval you want the manipulator to work for
qf_3 = p3;
qi_3 = p2;
qt_3 = []; %appendng the values computed in the for loop

q_dot_dot_c_3 = (4*(qf_3 - qi_3)/(tf_3)^2) *1.125 ;
tc_3 = tf_3/2 - 0.5* sqrt((((tf_3)^2 * norm(q_dot_dot_c_3))-(4 * norm(qf_3 - qi_3)))/norm(q_dot_dot_c_3));


for T = 0:0.001:1.4
    if (T>=0 && tc_3>=T)
        qt_3(end+1, :) = qi_3+ 0.5*q_dot_dot_c_3*T^2;
    elseif(T>tc_3 && T<=(tf_3 - tc_3))
        qt_3(end+1, :)=  qi_3+ q_dot_dot_c_3*tc_3*(T-tc_3/2);
    elseif(tf_3 - tc_3 <T && T<=tf_3)
        qt_3(end+1, :) =qf_3 - 0.5*(q_dot_dot_c_3)*(tf_3 - T)^2;
    end
    
end 

%Considering anticipation time for calculating arclength

%Segment3 anticipation time starts from 1.6 and ends at 3
% 0 to 1.6 = Segment3 will be 0
% After 3.0 , segment3 will remain constant

s3 = zeros(3401,1);
d3 = 0 ;
s3_temp = [0];
for i = 1:1399
   d3 = norm(qt_3(i+1,:)-qt_3(i,:)) + d3;
   s3_temp(end+1,:) = d3;
end
s3(1601:3000,1) = s3_temp(1:1400,1);
s3(3001:3401,1) = s3_temp(1400,1);

%Fourth Segment p3 to p4
tf_4 = t4 - t3; % 4 - 3.4 = 0.6 %tf is the time interval you want the manipulator to work for
qf_4 = p4;
qi_4 = p3;
qt_4 = []; %appendng the values computed in the for loop

q_dot_dot_c_4 = (4*(qf_4 - qi_4)/(tf_4)^2) *1.125 ;
tc_4 = tf_4/2 - 0.5* sqrt((((tf_4)^2 * norm(q_dot_dot_c_4))-(4 * norm(qf_4 - qi_4)))/norm(q_dot_dot_c_4));


for T = 0:0.001:0.6
    if (T>=0 && tc_4>=T)
        qt_4(end+1, :) = qi_4+ 0.5*q_dot_dot_c_4*T^2;
    elseif(T>tc_4 && T<=(tf_4 - tc_4))
        qt_4(end+1, :)=  qi_4+ q_dot_dot_c_4*tc_4*(T-tc_4/2);
    elseif(tf_4 - tc_4 <T && T<=tf_3)
        qt_4(end+1, :) =qf_4 - 0.5*(q_dot_dot_c_4)*(tf_4 - T)^2;
    end
    
end 

%Considering anticipation time for calculating arclength

%Segment4 anticipation time starts from 2.8 and ends at 3.4
% 0 to 2.8 = Segment4 will be 0
% End at 3.4seconds

s4 = zeros(3401,1);
d4 = 0 ;
s4_temp = [0];
for i = 1:600
   d4 = norm(qt_4(i+1,:)-qt_4(i,:)) + d4;
   s4_temp(end+1,:) = d4;
end
s4(2801:3401,1) = s4_temp(1:601,1);

% s = [s1(:,1);s2(:,1);s3(:,1);s4(:,1)];
% p=[p0,p1,p2,p3,p4];

first_s = ((s1(:,1)/norm(p1-p0)) * (p1-p0));
second_s = ((s2(:,1)/norm(p2-p1)) * (p2-p1));
third_s = ((s3(:,1)/norm(p3-p2)) * (p3-p2));
fourth_s = ((s4(:,1)/norm(p4-p3)) * (p4-p3));

pd = p0 + first_s + second_s + third_s + fourth_s;
    
figure(1)
plot(pd)
title('Position')
xlabel('Time')
ylabel('Position')
legend('x','y','z')

%velocity
    %Addding the last element of Pe_dot to Pe_dot
pd_dot = diff(pd/Ts);
pd_dot = [pd_dot; pd_dot(end,:)];

figure(2)
plot(pd_dot)
title('Velocity')
xlabel('Time')
ylabel('Velocity')
legend('x','y','z')

%Acceleration
pd_dot_dot = diff(pd_dot/Ts);
    %Addding the last element of Pe_dot_dot to Pe_dot_dot
pd_dot_dot = [pd_dot_dot ; pd_dot_dot(end,:)];

figure(3)
plot(pd_dot_dot)
title('Acceleration')
xlabel('Time')
ylabel('Acceleration')
legend('x','y','z')

figure(4)
plot3(pd(:,1),pd(:,2),pd(:,3))
title('3D Operational Space trajectory')


theta_d = zeros(3401,1);
theta_d_dot = zeros(3401,1);
theta_d_dot_dot = zeros(3401,1);

save('generated_traj')

