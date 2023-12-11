#! /usr/bin/python3

import numpy as np
from xarmrob_util.msg import ME439WaypointXYZ


def interpolate_position(start_xyz, end_xyz, v_max, acceleration, command_rate):
    """Smoothly interpolates between two positions using a constant-acceleration, maximum-velocity trajectory

    Args:
        start_xyz (np.array): An array of size (3,) which gives the starting position for movement interpolation in the
        form `[x, y, z]`
        end_xyz (np.array): An array of size (3,) which gives the ending position for movement interpolation in the
        form `[x, y, z]`
        v_max (float): The endpoint's maximum speed
        acceleration (float): The endpoint's acceleration
        command_rate (int or float): The rate at which commands are sent to the robot arm

    Yields:
        np.array: An array of size (3,) which gives the target position at each timestep
    """

    # Calculate the distance and direction we need to travel
    travel_vector = end_xyz - start_xyz
    travel_distance = np.linalg.norm(travel_vector)
    travel_direction = travel_vector / travel_distance

    # Calculate the time it will take to travel from the starting point to the end
    delta_t = 2 * np.sqrt(travel_distance / acceleration)

    # We might have far enough to travel that we hit v_max; if so, our delta_t will be longer
    t_v_max = v_max / acceleration
    if delta_t > 2 * t_v_max:
        delta_t = 2 * t_v_max + (travel_distance - acceleration * (t_v_max ** 2)) / v_max
    
    # Get each timestep's target position
    for t in np.arange(start=0, stop=delta_t, step=1/command_rate):

        # Accelerating towards the target point
        if t <= (delta_t / 2) and t <= t_v_max:
            delta_position = 1/2 * acceleration * (t ** 2)

        # Decelerating as the target point is reached
        elif t >= (delta_t / 2) and (delta_t - t) <= t_v_max:
            delta_position = travel_distance - 1/2 * acceleration * ((delta_t - t) ** 2)

        # Moving at v_max
        else:
            delta_position = 1/2 * acceleration * (t_v_max ** 2) + v_max * (t - t_v_max)

        # Construct the target position and yield it
        target_position = start_xyz + travel_direction * delta_position
        yield target_position


def interpolate_waypoints(waypoints, command_rate, start_xyz):
    """Given a set of waypoints and a starting position, yields a set of target positions which can be sent to the arm

    Args:
        waypoints (np.ndarray): An array of size (N, 5), where N is the number of waypoints to track. Each row of this
        matrix should be of th following form: `[target_x, target_y, target_z, v_max, acceleration]`.
        command_rate (int or float): The rate at which commands are sent to the robot arm
        start_xyz (np.array): An array of size (3,) which gives the starting position for waypoint interpolation in the
        form `[x, y, z]`

    Yields:
        ME439WaypointXYZ: The target location to move to in the current timestep
    """

    # Track the previous waypoint to travel from
    previous_waypoint_xyz = start_xyz

    # Iterate through each waypoint given
    target_position_waypoint = ME439WaypointXYZ()
    for waypoint in [waypoints[i, :] for i in range(waypoints.shape[0])]:

        # For each waypoint, calculate its trajectory using interpolation
        waypoint_xyz = waypoint[:3]
        waypoint_v_max = waypoint[3]
        waypoint_acceleration = waypoint[4]
        for target_position_xyz in interpolate_position(start_xyz=previous_waypoint_xyz,
                                                        end_xyz=waypoint_xyz,
                                                        v_max=waypoint_v_max,
                                                        acceleration=waypoint_acceleration,
                                                        command_rate=command_rate):
            target_position_waypoint.xyz = target_position_xyz
            yield target_position_waypoint
        
        # Update the previous waypoint location
        previous_waypoint_xyz = waypoint_xyz


if __name__ == '__main__':
    from parse_svg_for_drawing import convert_svg_to_waypoints

    svg_file = 'jerry_draw/src/smiley.svg'
    scaled_waypoints = convert_svg_to_waypoints(svg_file)

    for position in interpolate_waypoints(scaled_waypoints, command_rate=10, start_xyz=np.array([0, 0, 0])):
        print(position.xyz)