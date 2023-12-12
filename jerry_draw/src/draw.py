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


max_speed_move = rospy.get_param("/max_speed_move")
max_accel_move = rospy.get_param("/max_accel_move")
default_position_xyz = rospy.get_param("/default_position_xyz")
target_svg = rospy.get_param("/target_svg")
aruco_marker_id = rospy.get_param("/aruco_marker_id")
aruco_marker_averaging_samples = rospy.get_param("/aruco_marker_averaging_samples")
camera_pos_world = rospy.get_param("/camera_in_world")
camera_pitch = rospy.get_param("/camera_pitch")
command_frequency = rospy.get_param("/command_frequency")


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
    while not rospy.is_shutdown():
        for target_position in interpolate_waypoints(waypoints, command_frequency, default_position_xyz):
            pub_target_xyz.publish(target_position)
            rate.sleep()


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
    marker_position = get_marker_position(target_id=aruco_marker_id, num_samples=aruco_marker_averaging_samples)

    # Construct a transformation matrix to go from world-space to easel-space based on the marker's position
    easel_transform = aruco_marker_world_transform(marker_position, camera_pitch, camera_pos_world)

    # Apply this transformation to every waypoint
    waypoints = apply_transform(waypoints, easel_transform)

    # Add one last waypoint for Jerry to return to his default position when he finishes drawing
    waypoints = np.concatenate(waypoints, np.concatenate(default_position_xyz.reshape((1, -1)), [[max_speed_move, max_accel_move]], axis=1), axis=0)
    
    # Follow the listed waypoints smoothly
    follow_waypoints(waypoints)


# I'm a node!
rospy.init_node('drawer', anonymous=False)

# Start up main method automatically once a node is created
if __name__ == '__main__':
    main()