#! /usr/bin/python3
import numpy as np
import cv2
from cv2 import aruco
import pickle
import rospy
from jerry_draw.msg import aruco_marker_position
from jerry_draw.srv import get_aruco_marker_positon


# Aruco marker parameters: get these from a YAML file!
aruco_marker_size = 0.0508
aruco_marker_id = 42

# Load camera calibration info
cam_matrix = pickle.load(open("cam_matrix.p","rb"),encoding='bytes')
dist_matrix = pickle.load(open("dist_matrix.p","rb"),encoding='bytes')

# Tell OpenCV which ArUco tags we're using
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()


def scan_for_marker(req):
    # Extract parameters from the incoming request
    target_id = req.target_id
    num_samples = req.num_samples

    # Start a new video capture from the webcam
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT,960)

    # Display a preview until the user presses a key to start calibration
    while True:
        # Capture a frame and scan for ArUco markers
        _ = capture_frame(camera, preview_target_id=target_id)

        # Get key pressed while the displayed image is in focus
        key = cv2.waitKey(1)

        # If any key is pressed, start capturing marker info
        if (key != 255):
            cv2.destroyAllWindows()
            break
    
    # User has seen a preview, capture num_samples images containing our target marker
    marker_positions_with_target_id = []
    while len(marker_positions_with_target_id < num_samples):
        # Capture a frame of video and list all calculated marker positions
        visible_marker_positions = capture_frame(camera, preview=False)

        # See if any match our target ID
        for marker_position in visible_marker_positions:
            if marker_position.id == target_id:
                marker_positions_with_target_id.append(marker_position)
    
    # num_samples marker positions have been collected, average them together
    average_marker_pos = aruco_marker_position()
    average_marker_pos.id = target_id
    average_marker_pos.pos_x = np.mean([marker.pos_x for marker in marker_positions_with_target_id])
    average_marker_pos.pos_y = np.mean([marker.pos_y for marker in marker_positions_with_target_id])
    average_marker_pos.pos_z = np.mean([marker.pos_z for marker in marker_positions_with_target_id])
    average_marker_pos.rot_x = np.mean([marker.rot_x for marker in marker_positions_with_target_id])
    average_marker_pos.rot_y = np.mean([marker.rot_y for marker in marker_positions_with_target_id])
    average_marker_pos.rot_z = np.mean([marker.rot_z for marker in marker_positions_with_target_id])

    # Return the averaged samples
    return average_marker_pos


def capture_frame(camera, preview = True, preview_target_id = None):
    # Read a frame from the camera
    _, frame = camera.read()

    # Convert it to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Search for all ArUco markers in the captured image
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Process all the markers in the frame
    detected_markers = []
    for i_marker in range(len(corners)):
        # Estimate the 3D location of the marker
        rot_vec, trans_vec = aruco.estimatePoseSingleMarkers(corners[i_marker], aruco_marker_size, cam_matrix, dist_matrix)

        # Draw the marker axes on the frame (this gives a live preview)
        if preview:
            frame = aruco.drawAxis(frame, cam_matrix, dist_matrix, rot_vec, trans_vec, .1)

        # Store the calculated marker position
        current_marker_position = aruco_marker_position()
        current_marker_position.id = ids[i_marker]
        current_marker_position.pos_x = trans_vec[0][0][0]
        current_marker_position.pos_y = trans_vec[0][0][1]
        current_marker_position.pos_z = trans_vec[0][0][2]
        current_marker_position.rot_x = rot_vec[0][0][0]
        current_marker_position.rot_y = rot_vec[0][0][1]
        current_marker_position.rot_z = rot_vec[0][0][2]
        detected_markers.append(current_marker_position)

    # Add extra information to the debug preview
    if preview:
        # Draw outlines and ids for all the markers detected
        if len(corners) > 0:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # If a target ID is provided, tell the user whether or not it was detected in the current frame
        if preview_target_id is not None:
            if len(detected_markers) > 0 and preview_target_id in [id for id in detected_markers]:
                disp_msg = "ID " + str(preview_target_id) + " detected"
                frame = cv2.putText(frame, disp_msg, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 255, 64), 4)
            else:
                print([id for id in detected_markers], preview_target_id, detected_markers)
                disp_msg = "ID " + str(preview_target_id) + " not detected"
                frame = cv2.putText(frame, disp_msg, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 64, 64), 4)
        
    # Show the marked-up image
    if preview:
        cv2.imshow("ArUco Marker Detection", frame)

    # Return the detected marker positions
    return detected_markers


# Set up ROS service
rospy.init_node('aruco_scanner', anonymous=False)
service = rospy.Service('get_aruco_marker_positon', get_aruco_marker_positon, scan_for_marker)


# If this program is run directly, give an example
if __name__ == "__main__":
    rospy.wait_for_service('get_aruco_marker_positon')
    get_aruco_marker_positon = rospy.ServiceProxy('get_aruco_marker_positon', get_aruco_marker_positon)
    marker_position = get_aruco_marker_positon(target_id=42, num_samples=10)
    print("Found marker" + str(marker_position.id) + "at position" + str(marker_position.pos_x) + str(marker_position.pos_y) + str(marker_position.pos_z) + str(marker_position.rot_x) + str(marker_position.rot_y) + str(marker_position.rot_z))
