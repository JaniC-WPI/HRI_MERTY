% rosshutdown
% rosinit
clc; clear; close all

% set up dh_params
% start with symbolic representation of joint angles
syms q q0 q1 q2 q3 q4 q5 q6
q = [q0; q1; q2; q3; q4; q5; q6]; % arrange joints in a column vector

           % a       d     alpha  theta
dh_params = [0       0.333 0      q(1);
             0       0     -pi/2  q(2);
             0       0.316 pi/2   q(3);
             0.0825  0     pi/2   q(4);
             -0.0825 0.384 -pi/2  q(5);
             0       0     pi/2   q(6);
             0.088   0     pi/2   q(7)];

% subscribe to robot joint states 
%jointStateSub = rossubscriber("/franka_state_controller/joint_states",@jointStateCallback,"DataFormat","struct");
% publish desired joint velocities to PANDA:
%pandapub = rospublisher("/vs_dq3","std_msgs/Float64","DataFormat","struct");
         
% obtain symbolic homogeneous transform matrix of forward kinematics
pandahtm = htm(dh_params);
% isolate just the symbolic end effector position:
ee_xyz = pandahtm(1:3,4);
% set up an anonymous function that produces the 3x1 xyz position of end
% effector and takes inputs of the joint angles
get_ee_pos = matlabFunction(ee_xyz);
% set up Jacobian for velocity kinematics
J = jacobian(ee_xyz,q); % result is 3x7 Jacobian

get_ee_pos(0,0,0,0,0,0) % result seems to flip x and z axes

% generate a homogeneous transform matrix based on dh_params
function [T] = htm(dh_params)
    T = 0;
    for n = 1:size(dh_params,1)
        if T == 0
            T = TM(dh_params(n,:));
        else
            T = T * TM(dh_params(n,:));
        end
    end     
end

% return homogeneous transform matrix between two links of DH parameters
function T_x_y = TM(L)
    T_x_y = [cos(L(4)) -1*sin(L(4))*cos(L(3)) sin(L(4))*sin(L(3)) L(2)*cos(L(4));
             sin(L(4)) cos(L(4))*cos(L(3)) -1*cos(L(4))*sin(L(3)) L(2)*sin(L(4));
             0 sin(L(3)) cos(L(3)) L(1);
             0 0 0 1]; 
end