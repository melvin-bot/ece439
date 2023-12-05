import numpy as np

camera_angle = np.deg2rad(7)

Xa = 0.2
Ya = 0.3
Za = 0.9

C_rotate_x_pitch = np.array([[1, 0, 0, Xa],
                       [0, np.cos(camera_angle), -np.sin(camera_angle), Ya],
                       [0, np.sin(camera_angle), np.cos(camera_angle), Za],
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

print("Aruco in World frame:", "\n", M3)
