import numpy as np
# import rospy
from jerry_draw.msg import aruco_marker_position

# camera_angle = rospy.get_param("/camera_pitch")

def aruco_marker_world_pos(marker_pos_camera, camera_angle, camera_position):

    # The camera coordinates of the tag are weird; use the standard x-forward notation we've used previously
    pos_tag = np.array([[marker_pos_camera.z], [-marker_pos_camera.x], [-marker_pos_camera.y], [1]])

    # Translate from camera-space to world-space with a transform matrix
    camera_position = camera_position.reshape((-1))
    transform_camera_world = np.array(
        [[np.cos(camera_angle),   0,  np.sin(camera_angle),  camera_position[0]],
         [0,                      1,  0,                     camera_position[1]],
         [-np.sin(camera_angle),  0,  np.cos(camera_angle),  camera_position[2]],
         [0,                      0,  0,                     1]]
    )
    pos_world = transform_camera_world @ pos_tag

    # Pull out the x, y, z components of pos_world and return them
    return pos_world[:3, 0]


if __name__ == '__main__':
    test_aruco_position = aruco_marker_position()
    test_aruco_position.x = -0.05
    test_aruco_position.y = 0.0
    test_aruco_position.z = 0.46

    test_camera_angle = np.deg2rad(7)
    test_camera_position = np.array([-0.085, 0, 0.295])

    print("Aruco in World frame:", "\n", aruco_marker_world_pos(test_aruco_position, test_camera_angle, test_camera_position))
