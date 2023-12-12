#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from xarmrob_util.msg import ME439WaypointXYZ


# Dummy target
target = np.array([0.36,-0.28, 0.28])


# Kinematics parameters
l1 = rospy.get_param("/frame_offset_23")[0]
l3 = 0
l4 = rospy.get_param("/frame_offset_34")[2]
l5 = rospy.get_param("/frame_offset_34")[0]
x1 = rospy.get_param("/frame_offset_12")[0]
z1 = rospy.get_param("/frame_offset_01")[2]

dx_gripper = rospy.get_param("/endpoint_offset_in_frame_6")[0]
dz_gripper = rospy.get_param("/endpoint_offset_in_frame_6")[2]

gripper_hold_angle = rospy.get_param("/gripper_hold_angle")


# Constant parameters for inverse kinematics
d2 = np.sqrt(l4**2 + l5**2)
l2 = np.sqrt(dz_gripper**2 + (dx_gripper + d2 + l3)**2)


def pen_inverse_kinematics(target):
    target_x = target[0]
    target_y = target[1]
    target_z = target[2]

    alpha = np.arctan2(target_y, target_x)

    # find beta 2 prime
    delta_r = np.sqrt(target_x**2 + target_y**2) - x1
    delta_z = target_z - z1

    d = np.sqrt(delta_r**2 + delta_z**2)

    delta = np.arccos((l1**2 + l2**2 - d**2) / (2 * l1 * l2))
    beta2_prime = np.pi - delta

    # find beta 1
    psi = np.arctan2(delta_z, delta_r)
    phi = np.arccos((l1**2 + d**2 - l2**2) / 2 * l1 * d)

    beta1 = -1 * (psi + phi)

    # find beta 3
    beta3 = (np.pi / 2) - np.arctan2(l5, l4)

    # find beta2 error
    beta2_err = np.arctan2(-dz_gripper, (dx_gripper + d2 + l3))

    beta2 = beta2_prime - beta2_err

    return(alpha, beta1, beta2, beta3)


# JointState object to be re-used with each call to compute_inverse_kinematics
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper']

# Publisher for target joint states
pub_joint_state = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)


def compute_inverse_kinematics(msg_in):
    global joint_angles_desired_msg

    # Get the target position
    target_xyz = msg_in.xyz

    # Compute joint angles for that position
    alpha, beta1, beta2, beta3 = pen_inverse_kinematics(target_xyz)

    # Construct a set of joint angles
    angles = []
    angles.append(alpha)               # Base yaw
    angles.append(beta1)               # Shoulder pitch
    angles.append(beta2)               # Elbow pitch
    angles.append(0.0)                 # Elbow twist
    angles.append(beta3)               # Wrist pitch
    angles.append(0.0)                 # Writst twist
    angles.append(gripper_hold_angle)  # Gripper
    joint_angles_desired_msg.position = angles


    # Publish them so they can get executed
    pub_joint_state.publish(joint_angles_desired_msg)


# Subscriber for target endpoint position
sub_target_xyz = rospy.Subscriber('/target_xyz', ME439WaypointXYZ, compute_inverse_kinematics)


# Create a node for inverse kinematics to live in
if __name__ == '__main__':
    rospy.init_node('inverse_kinematics', anonymous=False)
    print("l1: " + str(l1))
    print("l3: " + str(l3))
    print("l4: " + str(l4))
    print("l5: " + str(l5))
    print("x1: " + str(x1))
    print("z1: " + str(z1))
    print("dx_gripper: " + str(dx_gripper))
    print("dz_gripper: " + str(dz_gripper))
    print("gripper_hold_angle: " + str(gripper_hold_angle))
    print("d2: " + str(d2))
    print("l2: " + str(l2))
    rospy.spin()