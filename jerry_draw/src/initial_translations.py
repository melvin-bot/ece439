import numpy as np
# import rospy
# from jerry_draw.msg import aruco_marker_position

# Get this from the YAML!
camera_angle = np.deg2rad(7)
# camera_angle = rospy.get_param("/camera_pitch")

def aruco_marker_world_pos(camera_pos, camera_angle):

    C_rotate_x_pitch = np.array([[1, 0, 0, camera_pos.x],
                        [0, np.cos(camera_angle), -np.sin(camera_angle), camera_pos.y],
                        [0, np.sin(camera_angle), np.cos(camera_angle), camera_pos.z],
                        [0, 0, 0, 1]])
    C_rotate_x = np.array([[1, 0, 0, 0],
                        [0, np.cos(np.pi/2), -np.sin(np.pi/2), 0],
                        [0, np.sin(np.pi/2), np.cos(np.pi/2), 0],
                        [0, 0, 0,1]])
    C_rotate_z = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0, 0],
                        [np.sin(np.pi/2), np.cos(np.pi/2), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    T_c_j = np.array([[0.085],
                    [0],
                    [-0.295],
                    [1]]
                    )

    M1 = np.matmul(C_rotate_x_pitch, C_rotate_x)
    M2 = np.matmul(M1, C_rotate_z)
    M3 = np.matmul(M2, T_c_j)

    return M3[:3, 0]


if __name__ == '__main__':
    class dummy:
        def __init__(self):
            pass

    test_aruco_position = dummy()
    test_aruco_position.x = -0.0513
    test_aruco_position.y = 0.0034
    test_aruco_position.z = 0.4645
    print("Aruco in World frame:", "\n", aruco_marker_world_pos(test_aruco_position, camera_angle))
