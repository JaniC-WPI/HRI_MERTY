function [phtm] = getPandaHTM(q, tftree)
% view available tf frames
%tftree.AvailableFrames;
    
%% obtain Atlas HTM

    % get transform from left_camera_optical_frame to left_camera_frame
    tf_0_1 = getTransform(tftree, 'panda_link0','panda_link1');
    T_0_1 = HTM_from_tf(tf_0_1)
    
    % get transform from left_camera_frame to head
    tf_1_2 = getTransform(tftree, 'panda_link1','panda_link2');
    T_1_2 = HTM_from_tf(tf_1_2)
    
    % get transform from right_camera_optical_frame to right_camera_frame
    tf_2_3 = getTransform(tftree, 'panda_link2','panda_link3');
    T_2_3 = HTM_from_tf(tf_2_3);

    % get transform from right_camera_frame to head
    tf_3_4 = getTransform(tftree,'panda_link3','panda_link4');
    T_3_4 = HTM_from_tf(tf_3_4);

    % get transform from head to utorso (q1)
    tf_4_5 = getTransform(tftree, 'panda_link4','panda_link5');
    T_4_5 = HTM_from_tf(tf_4_5);
    % this head-torso one is basically an Ry(q1) setup
    % thus, we make our symbolic HTM now
    % R1 = Ry_(q(1));
    % T_h_ut = [R1 T_head_utorso(1:3,4); 0 0 0 1];

    % get transform from utorso to l_clav Rz(q3)
    tf_5_6 = getTransform(tftree, 'panda_link5','panda_link6');
    T_5_6 = HTM_from_tf(tf_5_6); % this one is an Rz(q3)
%     T_1_3 = T_utorso_lclav;
%     R3 = Rz_(q(1)+pi);
%     T_1_3 = [R3 T_utorso_lclav(1:3,4); 0 0 0 1];
    
    
    
    % Panda HTM obtained
    phtm = T_0_1*T_1_2*T_4_5*T_1_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_8*T_8_9*T_llhand_lpalm*T_lpalm_lee;