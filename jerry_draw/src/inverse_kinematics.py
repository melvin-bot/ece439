#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from xarmrob_util.msg import ME439WaypointXYZ


# Kinematics parameters
l1 = rospy.get_param("/frame_offset_23")[0]
l4 = rospy.get_param("/frame_offset_34")[2]
l5 = rospy.get_param("/frame_offset_34")[0]
delta_r_origin = rospy.get_param("/frame_offset_12")[0]
delta_z_origin = rospy.get_param("/frame_offset_01")[2]
delta_x_gripper = rospy.get_param("/endpoint_offset_in_frame_6")[0]
delta_z_gripper = rospy.get_param("/endpoint_offset_in_frame_6")[2]
gripper_hold_angle = rospy.get_param("/gripper_hold_angle")

# l1 = 0.1005
# l4 = 0.0758
# l5 = 0.0627
# delta_r_origin = 0.010
# delta_z_origin = 0.074
# delta_x_gripper = 0.210
# delta_z_gripper = -0.025
# gripper_hold_angle = 0.05


# Constant parameters for inverse kinematics
D2 = np.linalg.norm([l4, l5])
l2 = np.linalg.norm([delta_z_gripper, (delta_x_gripper + D2)])

# There is a constant error term in beta_3 due to the simplifying assumptions
# we make to calculate beta_3_prime with a simple YPP robot
# Error term due to small angle between joint 3 and the pen tip
beta_3_err = np.arctan2((-1 * delta_z_gripper), (D2 + delta_x_gripper))
# Error term due to interior angle at joint 3
beta_3_err += np.arctan2(l5, l4)
# Need to add 90 degrees to beta_3 to account for zero position, subtract this from error term
beta_3_err -= np.pi / 2

def pen_inverse_kinematics(target):
    target_x = target[0]
    target_y = target[1]
    target_z = target[2]

    ####################
    # alpha_1
    alpha_1 = np.arctan2(target_y, target_x)

    ####################
    # beta_wrist
    beta_wrist = -1 * np.arctan2(l4, l5)

    ####################
    # beta_2
    delta_r_endpoint = np.linalg.norm([target_x, target_y]) - delta_r_origin
    delta_z_endpoint = target_z - delta_z_origin
    D = np.linalg.norm([delta_r_endpoint, delta_z_endpoint])

    psi = np.arctan2(delta_z_endpoint, delta_r_endpoint)
    phi = np.arccos((l1**2 + D**2 - l2**2) / (2 * l1 * D))

    beta_2 = -1 * (psi + phi)

    ####################
    # beta_3
    angle_delta = np.arccos((l1**2 + l2**2 - D**2) / (2 * l1 * l2))
    beta_3_prime = np.pi - angle_delta

    # l_error = l4 + np.cos(beta_wrist) * delta_z_gripper - np.sin(beta_wrist) * delta_x_gripper
    # beta_3_error = np.arccos(l_error / l2)

    beta_3 = beta_3_prime - beta_3_err
    # beta_3 = beta_3_prime - np.arctan2(delta_z_gripper, D2 + delta_x_gripper) - np.arctan2(l4, l5) + np.pi/2


    return(alpha_1, beta_2, beta_3, beta_wrist)


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
    alpha_1, beta_2, beta_3, beta_wrist = pen_inverse_kinematics(target_xyz)

    # Construct a set of joint angles
    angles = []
    angles.append(alpha_1)             # Base yaw
    angles.append(beta_2)              # Shoulder pitch
    angles.append(beta_3)              # Elbow pitch
    angles.append(0.0)                 # Elbow twist
    angles.append(beta_wrist)          # Wrist pitch
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
    print("l4: " + str(l4))
    print("l5: " + str(l5))
    print("x1: " + str(delta_r_origin))
    print("z1: " + str(delta_z_origin))
    print("dx_gripper: " + str(delta_x_gripper))
    print("dz_gripper: " + str(delta_z_gripper))
    print("gripper_hold_angle: " + str(gripper_hold_angle))
    print("D2: " + str(D2))
    print("l2: " + str(l2))
    rospy.spin()