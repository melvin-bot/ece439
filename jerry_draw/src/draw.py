#!/usr/bin/env python3

import sys
import numpy as np
import rospy
from parse_svg_for_drawing import convert_svg_to_waypoints
from easel_transform import aruco_marker_world_transform, apply_transform
from interpolate_waypoints import interpolate_waypoints
from xarmrob_util.msg import ME439WaypointXYZ
from jerry_draw.msg import aruco_marker_position
from jerry_draw.srv import get_aruco_marker_position
# from matplotlib import pyplot as plt


max_speed_move = rospy.get_param("/max_speed_move")
max_accel_move = rospy.get_param("/max_accel_move")
default_position_xyz = rospy.get_param("/default_position_xyz")
target_svg = rospy.get_param("/target_svg")
aruco_marker_id = rospy.get_param("/aruco_marker_id")
aruco_marker_averaging_samples = rospy.get_param("/aruco_marker_averaging_samples")
camera_pos_world = rospy.get_param("/camera_in_world")
camera_pitch = rospy.get_param("/camera_pitch")
command_frequency = rospy.get_param("/command_frequency")

# max_speed_move = 0.3
# max_accel_move = 1.0
# default_position_xyz = [0.1, -0.3, 0.1]
# target_svg = "/Users/Will/Desktop/Melvin/ece439/jerry_draw/src/smiley.svg"
# aruco_marker_id = 42
# aruco_marker_averaging_samples = 25
# camera_pos_world = [-0.085, 0, 0.295]
# camera_pitch = 0.122173
# command_frequency = 10


# Publisher for target endpoint locations
pub_target_xyz = rospy.Publisher('/target_xyz', ME439WaypointXYZ, queue_size=1)


def parse_drawing_method():
    if "--svg" in sys.argv:
        return "svg"
    elif "--raster" in sys.argv:
        return "raster"
    else:
        # Default to svg
        print("No valid drawing method passed (--svg or --raster); defaulting to svg")
        return "svg"


def follow_waypoints(waypoints):
    global pub_target_xyz

    # Publish target endpoint locations regularly
    rate = rospy.Rate(command_frequency)

    # Interpolate smoothly between all waypoints
    for target_position in interpolate_waypoints(waypoints, command_frequency, default_position_xyz):
        if not rospy.is_shutdown():
            pub_target_xyz.publish(target_position)
            rate.sleep()
    
    # return np.stack([target_position for target_position in interpolate_waypoints(waypoints, command_frequency, default_position_xyz)], axis=0)


def main():
    print("Started main draw method")
    global pub_target_xyz

    # Figure out if we are drawing an SVG or a raster image
    drawing_method = parse_drawing_method()

    # Generate a set of waypoints to follow
    if drawing_method == "svg":
        waypoints = convert_svg_to_waypoints(target_svg)
    elif drawing_method == "raster":
        raise NotImplementedError("Raster drawing is not yet implemented")

    # Move the arm to its starting position
    target_position = ME439WaypointXYZ()
    target_position.xyz = default_position_xyz
    pub_target_xyz.publish(target_position)

    # Using the camera, figure out where the AruCo marker is
    rospy.wait_for_service('get_aruco_marker_position')
    get_marker_position = rospy.ServiceProxy('get_aruco_marker_position', get_aruco_marker_position)
    marker_position = get_marker_position(target_id=aruco_marker_id, num_samples=aruco_marker_averaging_samples).marker_position

    # Construct a transformation matrix to go from world-space to easel-space based on the marker's position
    easel_transform = aruco_marker_world_transform(marker_position, camera_pitch, camera_pos_world)

    # Apply this transformation to every waypoint
    waypoints = apply_transform(waypoints, easel_transform)

    # Add one last waypoint for Jerry to return to his default position when he finishes drawing
    waypoints = np.concatenate((waypoints, np.concatenate((np.array(default_position_xyz).reshape((1, -1)), [[max_speed_move, max_accel_move]]), axis=1)), axis=0)
    
    # Follow the listed waypoints smoothly
    follow_waypoints(waypoints)

    # Debug method to enumerate transformations
    # for (x_new, y_new, z_new) in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]]:
    #     for (x_sign, y_sign, z_sign) in [[1, 1, 1], [1, 1, -1], [1, -1, 1], [1, -1, -1], [-1, 1, 1], [-1, 1, -1], [-1, -1, 1], [-1, -1, -1]]:
    #         # Reset waypoints
    #         waypoints = convert_svg_to_waypoints(target_svg)

    #         # Create another transformation from these parameters
    #         remap_transform = np.zeros((4, 4))
    #         remap_transform[0, x_new] = x_sign
    #         remap_transform[1, y_new] = y_sign
    #         remap_transform[2, z_new] = z_sign
    #         remap_transform[3, 3] = 1

    #         # Apply the transform remapping
    #         easel_transform_remapped = remap_transform @ easel_transform

    #         # Apply this transformation to every waypoint
    #         waypoints = apply_transform(waypoints, easel_transform_remapped)

    #         # Add one last waypoint for Jerry to return to his default position when he finishes drawing
    #         waypoints = np.concatenate((waypoints, np.concatenate((np.array(default_position_xyz).reshape((1, -1)), [[max_speed_move, max_accel_move]]), axis=1)), axis=0)
            
    #         # Follow the listed waypoints smoothly
    #         follow_waypoints(waypoints)

    #         # Plot the results
    #         title = f"[{x_new} * {x_sign}, {y_new} * {y_sign}, {z_new} * {z_sign}]"
    #         target_waypoints = follow_waypoints(waypoints)
    #         fig = plt.figure()
    #         fig.canvas.manager.set_window_title(title)
    #         ax = fig.add_subplot(projection='3d')
    #         ax.scatter(xs=target_waypoints[:,0], ys=target_waypoints[:,1], zs=target_waypoints[:,2])
    #         ax.set_aspect('equal')
    #         ax.set_xlabel('x')
    #         ax.set_ylabel('y')
    #         ax.set_zlabel('z')
    #         plt.show()


# I'm a node!
rospy.init_node('artist', anonymous=False)

# Start up main method automatically once a node is created
if __name__ == '__main__':
    main()