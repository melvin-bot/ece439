#!/usr/bin/env python3

import numpy as np
from jerry_draw.msg import aruco_marker_position
from cv2 import Rodrigues

def aruco_marker_world_pos(marker_pos_camera, camera_pitch, camera_yaw, camera_position):
    """Finds the position of an AruCo marker in world coordinates based on its posiiton in camera coordinates

    Args:
        marker_pos_camera (aruco_marker_position): The position of the AruCo marker, in camera coordinates
        camera_pitch (float): The downward pitch of the camera, in radians
        camera_position (np.array | np.ndarray): A row or column vector containing the (x, y, z) position of the camera
        in world coordinates

    Returns:
        np.array: A 3-unit vector containing the (x, y, z) posiiton of the AruCo marker, in world coordinates
    """    

    # The camera coordinates of the tag are weird; use the standard x-forward notation we've used previously
    pos_tag = np.array([[marker_pos_camera.pos_z], [-marker_pos_camera.pos_x], [-marker_pos_camera.pos_y], [1]])
    
    # Make camera_position into a numpy array, if needed
    camera_position = np.array(camera_position).reshape((-1))
    
    # Pitch the camera's view up or down
    rot_pitch = np.array([[np.cos(camera_pitch),   0,  np.sin(camera_pitch)],
                          [0,                      1,  0,                  ],
                          [-np.sin(camera_pitch),  0,  np.cos(camera_pitch)]])
    
    # Rotate about the camera's z axis to account for any yaw
    rot_yaw = np.array([[np.cos(camera_yaw), -np.sin(camera_yaw), 0],
                        [np.sin(camera_yaw),  np.cos(camera_yaw), 0],
                        [0,                   0,                  1]])
    
    # Apply both rotation matrixes, pitch is applied first
    rotation_matrix = rot_yaw @ rot_pitch

    # Construct a transform matrix to go from camera-space to world-space
    transform_camera_world = np.concatenate((rotation_matrix, camera_position.reshape((-1, 1))), axis=1)
    transform_camera_world = np.concatenate((transform_camera_world, np.array([[0, 0, 0, 1]])), axis=0)
    
    # Use this transform to locate the AruCo tag
    pos_world = transform_camera_world @ pos_tag

    # Pull out the x, y, z components of pos_world and return them
    return pos_world[:3, 0]


def aruco_marker_world_transform(marker_pos_camera, camera_pitch, camera_yaw, camera_position):
    """Construct a transformation matrix from world to easel coordinates. In the transformed coordinate space, +x is
    to the right, +y is up, and +z is out of the page of the easel; the origin is the AruCo tag.

    Args:
        marker_pos_camera (aruco_marker_position): The position of the AruCo marker, in camera coordinates
        camera_pitch (float): The downward pitch of the camera, in radians
        camera_position (np.array | np.ndarray): A row or column vector containing the (x, y, z) position of the camera
        in world coordinates

    Returns:
        np.ndarray: A 4x4 homogenous transform matrix
    """

    # Get the position of the AruCo tag, in world coordinates
    marker_pos_world = aruco_marker_world_pos(marker_pos_camera, camera_pitch, camera_yaw, camera_position)
    

    # Rotate about the y axis by the camera's pitch angle
    # rotation_pitch = np.array([[np.cos(camera_pitch), 0, np.sin(camera_pitch)],
    #                           [0, 1, 0],
    #                           [-np.sin(camera_pitch), 0, np.cos(camera_pitch)]])
    # rotation_matrix = rotation_pitch
    
    # Like in aruco_marker_world_pos, we rotate about the z axis to account for any yaw in the camera
    # rotation_yaw = np.array([[1, 0, 0],
    #                          [0, np.cos(camera_yaw), -np.sin(camera_yaw)],
    #                          [0, np.sin(camera_yaw),  np.cos(camera_yaw)]])
    # rotation_matrix = rotation_yaw @ rotation_matrix

    # The orientation of the AruCo tag is given by its Rodrigues vector
    # Like in aruco_marker_world_pos, we need to translate from camera coordinates to world coordinates
    # rodrigues_vector = np.array([marker_pos_camera.rot_x, marker_pos_camera.rot_y, marker_pos_camera.rot_z])
    # rotation_rodrigues, _ = Rodrigues(rodrigues_vector)
    # The Rodrigues rotation gives x to the right, y up, and z out of the page

    # Now we modify the Rodrigues vector to remap our coordinates to x up, y to the left, and z out of the page
    # rotation_rodruiges = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]]) @ rotation_rodrigues
    # rotation_matrix = rotation_rodrigues
    # rotation_matrix = rotation_rodruiges @ rotation_matrix
    
    
    # Remap all coordinates from svg space to camera space
    rotation_remap_svg = np.array(
            [[1, 0, 0],
             [0, -1, 0],
             [0, 0, -1]])
    
    # Remap coordinates to easel space in the camera's frame of reference
    rodrigues_vector = np.array([marker_pos_camera.rot_x, marker_pos_camera.rot_y, marker_pos_camera.rot_z])
    rotation_easel_camera, _ = Rodrigues(rodrigues_vector)
    
    # Remap easel-space coordinates in the camera's frame of reference to the world frame, taking into account the 180 degree rotation about x
    rotation_easel_world = np.array(
            [[0, 0, -1],
             [-1, 0, 0],
             [0, 1, 0]])
    # rotation_easel_world = np.array(
    #         [[0, 0, 1],
    #          [-1, 0, 0],
    #          [0, -1, 0]])
    
    # Pitch the camera up or down
    rotation_camera_pitch = np.array(
            [[np.cos(camera_pitch), 0, np.sin(camera_pitch)],
             [0, 1, 0],
             [-np.sin(camera_pitch), 0, np.cos(camera_pitch)]])
    
    # Yaw TODO
    
    # Construct the full rotation matrix
    rotation_matrix = rotation_camera_pitch @ (rotation_easel_world @ (rotation_easel_camera @ rotation_remap_svg))
    

    # Using the rotation matrix and world position vector, we can construct a combined homogenous transform
    # We have to swap the position vectors according to rotation_matrix for wizardry reasons
    transform = np.concatenate((rotation_matrix, marker_pos_world.reshape((-1, 1))), axis=1)
    transform = np.concatenate((transform, np.array([[0, 0, 0, 1]])), axis=0)

    # [z, x, -y]
    # transform = np.array([[-0.01008902 , 0.1076427  ,-0.99413845,  0.35335656],
    #                       [ 0.99993945 ,-0.00328239 ,-0.0105033 , -0.07481362],
    #                       [ 0.00439376 , 0.99418423 , 0.10760307,  0.20472887],
    #                       [ 0.         , 0.         , 0.        ,  1.        ],])

    return transform


def apply_transform(waypoints, transform):
    # New set of waypoints which have been transformed into easel space
    transformed_waypoints = np.empty_like(waypoints)

    for i_waypoint in range(waypoints.shape[0]):

        # Get the target waypoint
        target_waypoint = waypoints[i_waypoint, :]

        # Apply the transform to the waypoint's x/y/z coordinates
        waypoint_vector = np.array((target_waypoint[0], target_waypoint[1], target_waypoint[2], 1)).reshape((-1, 1))
        transformed_waypoint = (transform @ waypoint_vector).reshape((-1))[:3]

        # Add the transformed vector to the set of transformed waypoints
        transformed_waypoints[i_waypoint, :] = np.concatenate((transformed_waypoint, target_waypoint[3:]), axis=0)
    
    return transformed_waypoints


if __name__ == '__main__':
    test_aruco_position = aruco_marker_position()
    test_aruco_position.pos_x = -0.05
    test_aruco_position.pos_y = 0.0
    test_aruco_position.pos_z = 0.46

    test_camera_pitch = np.deg2rad(7)
    test_camera_yaw = 0.12
    test_camera_position = np.array([-0.085, 0, 0.295])

    print("Aruco in World frame:", "\n", aruco_marker_world_pos(test_aruco_position, test_camera_pitch, test_camera_yaw, test_camera_position))
