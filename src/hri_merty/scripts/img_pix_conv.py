#!/usr/bin/env python3.8

from Kinematics_mod_DH import Kinematics
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState
# from forward import Kinematics
from cv_bridge import CvBridge
import tf
import tf2_ros as tf2
import json
import os
from datetime import datetime


bridge = CvBridge()
camera_K = None
world_coords = None
img_pixels = None
image_pix = None
joint_vel = None
joint_angle = None

i = 0


def image_cb(img):
    print("image_callback getting called")
    global cv_img, bridge, i

    cv_img = bridge.imgmsg_to_cv2(img, "rgb8")

    cv2.imwrite("/home/fearless/Pictures/Data/nov1_v2/images/image" + str(i)+".jpg", cv_img)

    image_file = str(i)+".jpg"
    id = i
    data = {
        "objects": [
            {
                "class": "panda",
                "visibility": 1,
                "location": [
                    world_coords[0][0],
                    world_coords[0][0],
                    world_coords[0][0]
                ],
                "keypoints": [
                    {
                        "name": "panda_link0",
                        "location": [
                            world_coords[0][0],
                            world_coords[0][0],
                            world_coords[0][0]

                        ],
                        "projected_location": [
                            image_pix[0][0],
                            image_pix[0][1]
                        ]
                    },
                    {
                        "name": "panda_link2",
                        "location": [
                            world_coords[2][0],
                            world_coords[2][0],
                            world_coords[2][0]

                        ],
                        "projected_location": [
                            image_pix[2][0],
                            image_pix[2][1]
                        ]
                    },
                    {
                        "name": "panda_link3",
                        "location": [
                            world_coords[3][0],
                            world_coords[3][0],
                            world_coords[3][0]

                        ],
                        "projected_location": [
                            image_pix[3][0],
                            image_pix[3][1]
                        ]
                    },
                    {
                        "name": "panda_link4",
                        "location": [
                            world_coords[4][0],
                            world_coords[4][0],
                            world_coords[4][0]

                        ],
                        "projected_location": [
                            image_pix[4][0],
                            image_pix[4][1]
                        ]
                    },
                    {
                        "name": "panda_link6",
                        "location": [
                            world_coords[6][0],
                            world_coords[6][0],
                            world_coords[6][0]

                        ],
                        "projected_location": [
                            image_pix[6][0],
                            image_pix[6][1]
                        ]
                    },
                    {
                        "name": "panda_link7",
                        "location": [
                            world_coords[7][0],
                            world_coords[7][0],
                            world_coords[7][0]

                        ],
                        "projected_location": [
                            image_pix[7][0],
                            image_pix[7][1]
                        ]
                    }
                ]
            }
        ],
        "sim_state": {
            "joints": [
                {
                    "name": "panda_joint1",
                    "position": joint_angle[0],
                    "velocity": joint_vel[0]
                },
                {
                    "name": "panda_joint2",
                    "position": joint_angle[1],
                    "velocity": joint_vel[1]
                },
                {
                    "name": "panda_joint3",
                    "position": joint_angle[2],
                    "velocity": joint_vel[2]
                },
                {
                    "name": "panda_joint4",
                    "position": joint_angle[3],
                    "velocity": joint_vel[3]
                },
                {
                    "name": "panda_joint5",
                    "position": joint_angle[4],
                    "velocity": joint_vel[4]
                },
                {
                    "name": "panda_joint6",
                    "position": joint_angle[5],
                    "velocity": joint_vel[5]
                },
                {
                    "name": "panda_joint7",
                    "position": joint_angle[6],
                    "velocity": joint_vel[6]
                }
            ]
        }
    }

    json_obj = json.dumps(data, indent=4)
    filename = "/home/fearless/Pictures/Data/nov1_v2/images/image" + \
        str(i)+".json"
    with open(filename, "w") as outfile:
        outfile.write(json_obj)

    i = i + 1

#     cv2.circle(cv_img,(int(image_pix[0][0]), int(image_pix[0][1])),5,(0, 0, 255),-1)
#     cv2.circle(cv_img,(int(image_pix[1][0]), int(image_pix[1][1])),10,(0, 255, 0),-1)
#     cv2.circle(cv_img,(int(image_pix[2][0]), int(image_pix[2][1])),5,(255, 0, 0),-1)
#     cv2.circle(cv_img,(int(image_pix[3][0]), int(image_pix[3][1])),10,(255, 255, 0),-1)
#     cv2.circle(cv_img,(int(image_pix[4][0]), int(image_pix[4][1])),5,(0, 255, 255),-1)
#     cv2.circle(cv_img,(int(image_pix[5][0]), int(image_pix[5][1])),10,(255, 0, 255),-1)
#     cv2.circle(cv_img,(int(image_pix[6][0]), int(image_pix[6][1])),5,(255, 153, 255),-1)
#     cv2.circle(cv_img,(int(image_pix[7][0]), int(image_pix[7][1])),5,(150, 150, 0),-1)

#     i = i+1

#     cv2.imwrite("/home/jc-merlab/Pictures/2d_proj_ds/pix_test"+str(i)+".jpg", cv_img)


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
    # print("ht[6] = ", ht[6])

#     p_j0 = np.dot(ht0, np.transpose([0, 0, 0, 1]))
#     p_j1 = np.dot(ht1,np.transpose([0, 0, 0, 1]))
#     # print(p_j1)
#     p_j2 = np.dot(ht2, np.transpose([0, 0, 0, 1]))
#     # print(p_j2)
#     p_j3 = np.dot(ht3, np.transpose([0, 0, 0, 1]))
#     p_j4 = np.dot(ht4, np.transpose([0, 0, 0, 1]))
#     p_j5 = np.dot(ht5, np.transpose([0, 0, 0, 1]))
#     p_j6 = np.dot(ht6, np.transpose([0, 0, 0, 1]))
#     p_j7 = np.dot(ht7, np.transpose([0, 0, 0, 1]))

#     world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4, p_j5, p_j6, p_j7]

#     # print(p_j1,p_j2,p_j3,p_j4,p_j5,p_j6,p_j7)
#     return world_coords


def world_coords_tf(joints):
    global world_coords, joint_angle, joint_vel
    joint_angle = joints.position
    joint_vel = joints.velocity
    # print(type(joint_angle[0]))
    # print("Joint_angles", joint_angle)

    # joint_angle = joint_angle[0:7]

    kine = Kinematics(joint_angle)

    print(kine)
    ht = kine.forward()
    # print("Forward Kinematics Transform", ht)

    p_j0 = np.dot(np.eye(4), np.transpose([0, 0, 0, 1]))
    p_j1 = np.dot(ht[0], np.transpose([0, 0, 0, 1]))
    # print(p_j1)
    p_j2 = np.dot(ht[1], np.transpose([0, 0, 0, 1]))
    # print(p_j2)
    p_j3 = np.dot(ht[2], np.transpose([0, 0, 0, 1]))
    p_j4 = np.dot(ht[3], np.transpose([0, 0, 0, 1]))
    p_j5 = np.dot(ht[4], np.transpose([0, 0, 0, 1]))
    p_j6 = np.dot(ht[5], np.transpose([0, 0, 0, 1]))
    p_j7 = np.dot(ht[6], np.transpose([0, 0, 0, 1]))

    world_coords = [p_j0, p_j1, p_j2, p_j3, p_j4, p_j5, p_j6, p_j7]

    print("world_coords", world_coords)


def camera_intrinsics(camera_info):
    # Create camera intrinsics matrix
    global camera_K
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    camera_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


# homogenous tranformation from 4X1 translation and


def transform(tvec, quat):

    r = R.from_quat(quat).as_matrix()

#     print("r", r)

#     r = R.from_quat(quat).as_dcm()
    ht = np.array([[r[0][0], r[0][1], r[0][2], tvec[0]],
                   [r[1][0], r[1][1], r[1][2], tvec[1]],
                   [r[2][0], r[2][1], r[2][2], tvec[2]],
                   [0, 0, 0, 1]])

    return ht


# using the camera extrinsics from DREAM, and the world coords from tf transformation in this function we are creating the
def image_pixels(camera_ext, world_coords):
    global img_pixels

    print("Camera Extrinsics", camera_K)
    print("is image pixels getting called")
    proj_model = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]
    image_pix1 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[0]))
    image_pix2 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[1]))
    image_pix3 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[2]))
    image_pix4 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[3]))
    image_pix5 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[4]))
    image_pix6 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[5]))
    image_pix7 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[6]))
    image_pix8 = np.dot(np.dot(camera_K, proj_model),
                        np.dot(camera_ext, world_coords[7]))

    u1 = image_pix1[0]/image_pix1[2]
    v1 = image_pix1[1]/image_pix1[2]

    # u1 = image_pix1[0]
    # v1 = image_pix1[1]

    u2 = image_pix2[0]/image_pix2[2]
    v2 = image_pix2[1]/image_pix2[2]

    # u2 = image_pix2[0]
    # v2 = image_pix2[1]

    u3 = image_pix3[0]/image_pix3[2]
    v3 = image_pix3[1]/image_pix3[2]

    # u3 = image_pix3[0]
    # v3 = image_pix3[1]

    u4 = image_pix4[0]/image_pix4[2]
    v4 = image_pix4[1]/image_pix4[2]

    # u4 = image_pix4[0]
    # v4 = image_pix4[1]

    u5 = image_pix5[0]/image_pix5[2]
    v5 = image_pix5[1]/image_pix5[2]

    # u5 = image_pix5[0]
    # v5 = image_pix5[1]

    u6 = image_pix6[0]/image_pix6[2]
    v6 = image_pix6[1]/image_pix6[2]

    # u6 = image_pix6[0]
    # v6 = image_pix6[1]

    u7 = image_pix7[0]/image_pix7[2]
    v7 = image_pix7[1]/image_pix7[2]

    # u7 = image_pix7[0]
    # v7 = image_pix7[1]

    u8 = image_pix8[0]/image_pix8[2]
    v8 = image_pix8[1]/image_pix8[2]

    # u8 = image_pix8[0]
    # v8 = image_pix8[1]

    img_pixels = [(u1, v1), (u2, v2), (u3, v3), (u4, v4),
                  (u5, v5), (u6, v6), (u7, v7), (u8, v8)]

    print(img_pixels)

    return img_pixels


def main():
    global image_pix
    # Initialize the node
    rospy.init_node('image_pix_gen')
    print("is main getting called")
    # subscriber for rgb image to detect markers
    image_sub = rospy.Subscriber(
        "/camera/color/image_raw", Image, image_cb, queue_size=1)
    cam_info_sub = rospy.Subscriber(
        "/camera/color/camera_info", CameraInfo, camera_intrinsics, queue_size=1)
    joint_sub = rospy.Subscriber(
        "/franka_state_controller/joint_states", JointState, world_coords_tf, queue_size=1)

    # tfBuffer = tf2.Buffer()
    # listener = tf2.TransformListener(tfBuffer)

    # tf_1 = tfBuffer.lookup_transform('panda_link0', 'panda_link1', rospy.Time(0), rospy.Duration(1.0))
    # tf_2 = tfBuffer.lookup_transform('panda_link0', 'panda_link2', rospy.Time(0), rospy.Duration(1.0))
    # tf_3 = tfBuffer.lookup_transform('panda_link0', 'panda_link3', rospy.Time(0), rospy.Duration(1.0))
    # tf_4 = tfBuffer.lookup_transform('panda_link0', 'panda_link4', rospy.Time(0), rospy.Duration(1.0))
    # tf_5 = tfBuffer.lookup_transform('panda_link0', 'panda_link5', rospy.Time(0), rospy.Duration(1.0))
    # tf_6 = tfBuffer.lookup_transform('panda_link0', 'panda_link6', rospy.Time(0), rospy.Duration(1.0))
    # tf_7 = tfBuffer.lookup_transform('panda_link0', 'panda_link7', rospy.Time(0), rospy.Duration(1.0))

    # tvec = [-0.36764867,  0.48387939,  1.57487899]
    # quat = [0.71465954, -0.10872412,  0.06709603,  0.68770555]

    # # oct25_v1
    # tvec = [-0.12299873,  0.5138326,   1.67000566]
    # quat = [0.6753712,   0.0565828,  -0.09733649,  0.72883313]

    # # oct25_v2
    # tvec = [-0.31601359,  0.52107275,  1.67631562]
    # quat = [0.6715593,   0.04552082, -0.08661441,  0.73446164]

    # # # oct25_v3
    # tvec = [-0.28643136,  0.51100448,  1.73800231]
    # quat = [0.6816269,   0.04557668, -0.09043956,  0.72465731]

    # # # oct25_v4
    # tvec = [-0.21411439,  0.52268317, 1.64100121]
    # quat = [0.67768642, -0.0063368, -0.03515476, 0.73448288]

    # # oct25_v5
    # tvec = [-0.23717485,  0.51588339,  1.58997365]
    # quat = [0.66575701,  0.02722918, -0.06353728,  0.74295975]

    # tvec = [-0.13785074,  0.5225022,   1.87940903]
    # quat = [0.69354447,  0.05959975, -0.10355144,  0.71043721]

    # nov1_v1
    tvec = [-0.16541568,  0.52310531,  1.67225892]
    quat = [0.66345224,  0.09280906, -0.13007338,  0.73095726]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if camera_K is not None:
            camera_ext = transform(tvec, quat)
            print("camera_ext", camera_ext)
            image_pix = image_pixels(camera_ext, world_coords)
        rate.sleep()
        print(image_pix)

    rospy.spin()


if __name__ == "__main__":
    main()
