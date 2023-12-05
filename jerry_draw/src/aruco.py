#! /usr/bin/python3
import numpy as np
import cv2
import PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pickle
import rospy
from std_msgs.msg import String

#Setup camera capture and resolution
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,960)

# What is the size of each marker - length of a side in meters (or any other unit you are working with). Used in call to "estimatePoseSingleMarkers". 
marker_side_length = 0.0508 # meters 

#comment this out to remove live display (and some other stuff below)
plt.figure()
#Load calibration from pickle files (Python2 format..  you'll have to recalibrate if you use another camera)
cam_matrix = pickle.load(open("cam_matrix.p","rb"),encoding='bytes')
dist_matrix = pickle.load(open("dist_matrix.p","rb"),encoding='bytes')

#setup ROS stuff
rospy.init_node('aruco_node', anonymous=False)
pub_aruco = rospy.Publisher('/aruco', String, queue_size=10)

#Tell opencv which aruco tags we're using (this should match the generation script)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

#Loop until a key gets pressed on the video
while True:
    #Read a frame from the camera
    retval, frame = camera.read()
    #convert to grayscale
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #try to find fiducials in the image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
    if (len(corners) > 0):
        toSend = ""
        #loop through all the detected markers
        for i in range(0,len(corners)):
            #Estimate the 3D location of the marker
            rot_vec, trans_vec = aruco.estimatePoseSingleMarkers(corners[i],marker_side_length,cam_matrix,dist_matrix);
            axis = np.float32([[4,0,0],[0,4,0],[0,0,-4]]).reshape(-1,3)
            #Calculate coordinates in the image of the markers
            imgpts, jac = cv2.projectPoints(axis,rot_vec,trans_vec,cam_matrix,dist_matrix);
            #draw the axes for the markers (remove for no live display)
            frame = aruco.drawAxis(frame,cam_matrix,dist_matrix,rot_vec,trans_vec,.1)
            #rotMat = cv2.Rodrigues(rot_vec);
            #print(rotMat)
            #String to send through the topic
            toSend = toSend +  "ID:%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f" % (ids[i],trans_vec[0][0][0], trans_vec[0][0][1], trans_vec[0][0][2],rot_vec[0][0][0],rot_vec[0][0][1],rot_vec[0][0][2])
            #Only display if one tag is shown (opencv can't do \n...)
            if(len(corners) == 1):
                frame = cv2.putText(frame,toSend,(30,30),cv2.FONT_HERSHEY_SIMPLEX,1,(255,200,75),4)
        #draw outlines and ids for all the markers detected. (remove for no live display)
        frame = aruco.drawDetectedMarkers(frame.copy(),corners,ids)
        #Send ROS message
        pub_aruco.publish(toSend)
    #display the image with debug info (remove for no live display)
    cv2.imshow("live video", frame)
    #Get keys pushed when the image is in focus (ctr-c works if there isn't live display)
    key = cv2.waitKey(1)
    #If any key is pushed, quit the program
    if (key != 255):
        cv2.destroyAllWindows()
        exit()
