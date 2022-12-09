# HRI_MERTY
1. git clone the project
2. cd HRI_MERTY
3. catkin_make
4. Identify Object and pick up
  - python voice_feedback_rospy_node.py
  - roslaunch hri_merty franka_control_service.launch
5. Identify palm and place object on palm
  - python voice_feedback_rospy_node.py
  - python identify_palm.py
  - python franka_skel_grasp.py
