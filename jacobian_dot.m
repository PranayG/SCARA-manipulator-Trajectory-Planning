function [S] = jacobian_dot(q,q_dot)

    theta1 = q(1);
    theta2 = q(2);

    theta1_dot = q_dot(1);
    theta2_dot = q_dot(2);
    
    %d3_dot = q_dot(3);
    %theta4_dot = q_dot(4);
    
    Jdot = [-0.5*cos((theta1))*(theta1_dot)-0.5*cos((theta1+theta2))*(theta1_dot+theta2_dot) , -0.5*cos((theta1+theta2))*(theta1_dot+theta2_dot), 0 ,0; 
           -0.5*sin((theta1))*(theta1_dot)-0.5*sin((theta1+theta2))*(theta1_dot+theta2_dot) , -0.5*sin((theta1+theta2))*(theta1_dot+theta2_dot), 0 ,0;
           0, 0, 0, 0 ; 
           0, 0, 0, 0];

    S = Jdot*q_dot;

    S = S';
end