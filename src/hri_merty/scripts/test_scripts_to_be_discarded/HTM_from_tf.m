function [T_] = HTM_from_tf(tf_)
    t_ = [tf_.Transform.Translation.X;
              tf_.Transform.Translation.Y;
              tf_.Transform.Translation.Z];
    quat_ = tf_.Transform.Rotation;
    R_ = quat2rotm([quat_.W quat_.X quat_.Y quat_.Z]);
    T_ = [R_ t_; 0 0 0 1];
end