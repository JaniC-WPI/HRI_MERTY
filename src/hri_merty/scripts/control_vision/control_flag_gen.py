import rospy
import numpy as np
from std_msgs.msg import String, Bool

def main():
    rospy.init_node("start_color_flag", anonymous=True)
    color_pub = rospy.Publisher("/color/flag", String, queue_size=1)
    control_pub = rospy.Publisher('/franka/control_flag', Bool, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        color_pub.publish("red")
        # control_pub.publish(True)
        rate.sleep()
    
    rospy.spin()

if __name__=='__main__':
    main()