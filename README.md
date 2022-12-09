# HRI_MERTY
1. Identify Object and pick up
python voice_feedback_rospy_node.py
roslaunch franka_control_service.launch

2. Identify palm and place object on palm
python voice_feedback_rospy_node.py
python identify_palm.py
python franka_skel_grasp.py
