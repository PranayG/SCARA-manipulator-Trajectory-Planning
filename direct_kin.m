%There is a base fram and it is not coincident to the Zero'th frame so we
%consider another Homogenous Matrix with Pure Translation along d0 (Z-axis)

%Direct Kinematics is considered as the Homogenous Matrix of the end
%effector with respect to the base frame.

%q are the Joint variables (theta1 , theta2, d3 and theta4)that is computed
%when the simulink model is run and then it is given as inputs to
%direct_kin equation to correct the error.

function [DK_eq] = direct_kin(q)
theta1 = q(1);
theta2 = q(2);
d3 = q(3);
theta4 = q(4);


%Base to Zero frame 
%d0 =1m 
HB_0 = DH(1, 0, 0 ,0);
%link1 
%a1 = Length of the link 1= 0.5m
H0_1 = DH(0 , 0 , theta1 , 0.5);
%link2  , a2 = Length of the Link2 = 0.5m
H1_2 = DH(0 , 0 , theta2, 0.5);
%link3
H2_3 = DH(-d3,0 ,0 ,0);   %d3 travels along -z2 axis in the manipulator
%Link4
H3_4 = DH(0 , 0 , theta4, 0);
%Frame1 wrt Base frame
%HB_1 = simplify(HB_0 * H0_1);
%Frame2 wrt Base frame
%HB_2 = simplify(HB_0*H0_1*H1_2);
%Frame3 wrt Base
% frame
%HB_3 = simplify(HB_0*H0_1*H1_2*H2_3);

%Direct Kinematics (Final Homogenous Matrix) Base to End Effector
HB_4 = HB_0 * H0_1 * H1_2 * H2_3 * H3_4;

DK_eq = [HB_4(1,4); HB_4(2,4); HB_4(3,4); (theta1+theta2+theta4)];
%DK_eq = subs(DK_eq ,[theta1  theta2 d3 theta4] , [q(1), q(2), q(3), q(4)]);
DK_eq = double(DK_eq');

end


function  [A] = DH(d , alpha , theta, a)
A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
     0 sin(alpha) cos(alpha) d ;
     0  0  0 1 ];  
end




    
    