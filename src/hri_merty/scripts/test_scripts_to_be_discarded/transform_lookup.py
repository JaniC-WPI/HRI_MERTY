import tf
import tf2_ros as tf2
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy


def transform(tvec, quat):

    r = R.from_quat(quat).as_matrix()

#     print("r", r)

#     r = R.from_quat(quat).as_dcm()
    ht = np.array([[r[0][0], r[0][1], r[0][2], tvec[0]],
                   [r[1][0], r[1][1], r[1][2], tvec[1]],
                   [r[2][0], r[2][1], r[2][2], tvec[2]],
                   [0, 0, 0, 1]])

    return ht

def world_coords_tf():

    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)

    t0 = [0, 0, 0]
    r0 = [0, 0, 0, 1]

    ht0 = transform(t0, r0)

    print("ht0 = ", ht0)

    tf_1 = tfBuffer.lookup_transform('panda_link0', 'panda_link1', rospy.Time(0), rospy.Duration(1.0))

    t1 = [tf_1.transform.translation.x, tf_1.transform.translation.y, tf_1.transform.translation.z]
    r1 = [tf_1.transform.rotation.x, tf_1.transform.rotation.y, tf_1.transform.rotation.z, tf_1.transform.rotation.w]


    ht1 = transform(t1, r1)

    print("ht1 = ", ht1)
    # print("ht[0]", ht[0])

    tf_2 = tfBuffer.lookup_transform('panda_link0', 'panda_link2', rospy.Time(0), rospy.Duration(1.0))

    t2 = [tf_2.transform.translation.x, tf_2.transform.translation.y, tf_2.transform.translation.z]
    r2 = [tf_2.transform.rotation.x, tf_2.transform.rotation.y, tf_2.transform.rotation.z, tf_2.transform.rotation.w]


    ht2 = transform(t2, r2)

    print("ht2 = ", ht2)
    # print("ht[1] = ", ht[1])

    tf_3 = tfBuffer.lookup_transform('panda_link0', 'panda_link3', rospy.Time(0), rospy.Duration(1.0))

    t3 = [tf_3.transform.translation.x, tf_3.transform.translation.y, tf_3.transform.translation.z]
    r3 = [tf_3.transform.rotation.x, tf_3.transform.rotation.y, tf_3.transform.rotation.z, tf_3.transform.rotation.w]

    ht3 = transform(t3, r3)

    print("ht3 = ", ht3)
    # print("ht[2]= ", ht[2])

    tf_4 = tfBuffer.lookup_transform('panda_link0', 'panda_link4', rospy.Time(0), rospy.Duration(1.0))

    t4 = [tf_4.transform.translation.x, tf_4.transform.translation.y, tf_4.transform.translation.z]
    r4 = [tf_4.transform.rotation.x, tf_4.transform.rotation.y, tf_4.transform.rotation.z, tf_4.transform.rotation.w]

    ht4 = transform(t4, r4)

    print("ht4 = ", ht4)
    # print("ht[3] = ", ht[3])

    tf_5 = tfBuffer.lookup_transform('panda_link0', 'panda_link5', rospy.Time(0), rospy.Duration(1.0))

    t5 = [tf_5.transform.translation.x, tf_5.transform.translation.y, tf_5.transform.translation.z]
    r5 = [tf_5.transform.rotation.x, tf_5.transform.rotation.y, tf_5.transform.rotation.z, tf_5.transform.rotation.w]

    ht5 = transform(t5, r5)

    print("ht5 = ", ht5)
    # print("ht[4] = ", ht[4])

    tf_6 = tfBuffer.lookup_transform('panda_link0', 'panda_link6', rospy.Time(0), rospy.Duration(1.0))

    t6 = [tf_6.transform.translation.x, tf_6.transform.translation.y, tf_6.transform.translation.z]
    r6 = [tf_6.transform.rotation.x, tf_6.transform.rotation.y, tf_6.transform.rotation.z, tf_6.transform.rotation.w]

    ht6 = transform(t6, r6)

    print("ht6 = ", ht6)
    # print("ht[5] = ", ht[5])

    tf_7 = tfBuffer.lookup_transform('panda_link0', 'panda_link7', rospy.Time(0), rospy.Duration(1.0))

    t7 = [tf_7.transform.translation.x, tf_7.transform.translation.y, tf_7.transform.translation.z]
    r7 = [tf_7.transform.rotation.x, tf_7.transform.rotation.y, tf_7.transform.rotation.z, tf_7.transform.rotation.w]

    ht7 = transform(t7, r7)

    print("ht7 = ", ht7)