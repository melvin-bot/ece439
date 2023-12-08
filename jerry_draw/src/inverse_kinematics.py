import numpy as np
import rospy

target = np.array([0.36,-0.28, 0.28])

l1 = rospy.get_param("/frame_offset_23")[0]
l3 = rospy.get_param("/frame_offset_23")[0]
l4 = rospy.get_param("/frame_offset_34")[0]
l5 = rospy.get_param("/frame_offset_34")[2]
x1 = rospy.get_param("/frame_offset_12")[0]
z1 = rospy.get_param("/frame_offset_01")[2]

d2 = np.sqrt(l4**2 + l5**2)

dx_gripper = rospy.get_param("/endpoint_offset_in_frame_6")[0]
dz_gripper = rospy.get_param("/endpoint_offset_in_frame_6")[2]

l2 = np.sqrt(-dz_gripper**2 + (dx_gripper + d2 + l3)**2)

def pen_inverse_kinematics(target):
    target_x = target[0]
    target_y = target[1]
    target_z = target[2]

    alpha = np.arctan2(target_y / target_x)

    # find beta 2 prime
    delta_r = np.sqrt(target_x**2 + target_y**2) - x1
    delta_z = target_z - z1

    d = np.sqrt(delta_r**2 + delta_z**2)

    delta = np.arccos((l1**2 + l2**2 - d**2) / (2 * l1 * l2))
    beta2_prime = np.pi - delta

    # find beta 1
    psi = np.arctan2(delta_z/delta_r)
    phi = np.arccos((l1**2 + d**2 - l2**2) / 2 * l1 * d)

    beta1 = psi + phi

    # find beta 3
    beta3 = (np.pi / 2) - np.arctan2(l5/l4)

    # find beta2 error
    beta2_err = np.arctan2(-dz_gripper / (dx_gripper + d2 + l3))

    beta2 = beta2_prime - beta2_err

    return([alpha, beta1, beta2, beta3])