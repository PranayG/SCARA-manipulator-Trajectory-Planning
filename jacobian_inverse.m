function [q_dot_dot] = jacobian_inverse(xdDot_Kde_dot_Kpe_Jdot_q_dot, q)
    theta1 = q(1);
    theta2 = q(2);

     
     %theta1_dot = q_dot(1);
     %theta2_dot = q_dot(2);
    
    [IndJ,~] = jacobian(q,0);

    q_dot_dot = IndJ\(xdDot_Kde_dot_Kpe_Jdot_q_dot);
end

