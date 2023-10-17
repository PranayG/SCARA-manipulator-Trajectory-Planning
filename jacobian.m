function [IndJ,xe_dot] = jacobian(q,q_dot)
    theta1 = q(1);
    theta2 = q(2);

    %theta1_dot = q_dot(1);
    %theta2_dot = q_dot(2);
    
    J = [-0.5*sin(theta1)-0.5*sin(theta1 + theta2) -0.5*sin(theta1+theta2) 0 0;
      0.5*cos(theta1)+0.5*cos(theta1+theta2) 0.5*cos(theta1+theta2) 0 0;
      0 0 -1 0;
      0 0 0 0 ;
      0 0 0 0;
      1 1 0 1];

    %To find the inverse(Jacobian) we find the dependant rows and eliminate
    %them. We have 2 zero rows and 4 independant rows. Thus we remove the 2
    %zeroes

    IndJ = [-0.5*sin(theta1)-0.5*sin(theta1 + theta2) -0.5*sin(theta1+theta2) 0 0;
      0.5*cos(theta1)+0.5*cos(theta1+theta2) 0.5*cos(theta1+theta2) 0 0;
      0 0 -1 0;
      1 1 0 1];

    xe_dot = IndJ * q_dot;

    xe_dot = xe_dot';
end