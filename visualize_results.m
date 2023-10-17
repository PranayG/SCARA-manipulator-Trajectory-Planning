function y = visualize_results(t, q, error_pd_dot, error_pd)
    
%     tracking = tracking';
%     tracking_theta = tracking_theta';
    addpath("visualization/")
    
    %Joint Values
    theta_1 = q(1,:,:);
    theta_2 = q(2,:,:);
    d_3 = q(3,:,:);
    theta_4 = q(4,:,:);

    theta1(:,1) = theta_1(1,1,:);
    theta2(:,1) = theta_2(1,1,:);
    d3(:,1) = d_3(1,1,:);
    theta4(:,1) = theta_4(1,1,:);
    
    %Position error
    pos1 = error_pd(1,:,:);
    pos2 = error_pd(2,:,:);
    pos3 = error_pd(3,:,:);
    pos4 = error_pd(4,:,:);

    e_pd_1(:,1) = pos1(1,1,:);
    e_pd_2(:,1) = pos2(1,1,:);
    e_pd_3(:,1) = pos3(1,1,:);
    e_pd_4(:,1) = pos4(1,1,:);

    %Velocity Error
    pd_dot_q1 = error_pd_dot(1,:,:);
    pd_dot_q2 = error_pd_dot(2,:,:);
    pd_dot_q3 = error_pd_dot(3,:,:);
    pd_dot_q4= error_pd_dot(4,:,:);

    ve1(:,1) = pd_dot_q1(1,1,:);
    ve2(:,1) = pd_dot_q2(1,1,:);
    ve3(:,1) = pd_dot_q3(1,1,:);
    ve4(:,1) = pd_dot_q4(1,1,:);
    
    
   figure('Name' , 'Joint Space Position Errors')
    subplot(5,1,1); plot(t, e_pd_1);
    title('Position Error Joint1');
    subplot(5,1,2); plot(t, e_pd_2);
    title('Position Error Joint2');
    subplot(5,1,3); plot(t, e_pd_3);
    title('Position Error Joint3');
    subplot(5,1,4); plot(t, e_pd_4);
    title('Position Error Joint4');
    subplot(5,1,5);
    title('SCARA');

   figure('Name' , 'Joint Space Velocity Errors')
    subplot(5,1,1); plot(t, ve1);
    title('Vel Error Joint1');
    subplot(5,1,2); plot(t, ve2);
   title('Vel Error Joint2');
    subplot(5,1,3); plot(t, ve3);
    title('Vel Error Joint3');
    subplot(5,1,4); plot(t, ve4);
    title('Vel Error Joint4');
    subplot(5,1,5);
    title('SCARA');
   

   figure('Name' ,'Joint Trajectories')
    subplot(5,1,1); plot(t, theta1);
    title('Joint1');
    subplot(5,1,2); plot(t, theta2);
    title('Joint2');
    subplot(5,1,3); plot(t, d3);
    title('Joint3');
    subplot(5,1,4); plot(t, theta4);
    title('Joint4');
    subplot(5,1,5);
    title('SCARA');

end