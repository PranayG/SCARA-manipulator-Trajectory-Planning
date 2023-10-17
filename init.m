clear all
close all
clc
vrclear
vrclose

load('generated_traj.mat');
control;
sim('control.mdl', t);
visualize_results(t,q,error_pd_dot,error_pd);
SCARA_VR_VISUALIZE(squeeze(q(:,1,:)), false);

