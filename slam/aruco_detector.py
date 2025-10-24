# detect ARUCO markers and estimate their positions
import numpy as np
import cv2
import os, sys

sys.path.insert(0, "{}/util".format(os.getcwd()))
import util.measure as measure

# python3 SLAM_eval.py lab_output/MAPTEST2.txt lab_output/slam.txt

class aruco_detector:
    def __init__(self, robot, marker_length=0.7):#TODO change this back to 0.7!!!!
        
        self.camera_matrix = robot.camera_matrix
        self.distortion_params = robot.camera_dist
        
        # offset =  input("enter the offset")
        
        # offset = float(offset)

        # self.marker_length = marker_length-0.0102#was 0.0101
        self.marker_length = marker_length#- 0.010#w0.0101
        
        #for marker size 0.7, use 0.008 or if closer distnace -0.012
        #for marker size 0.68 use -0.0101

        self.aruco_params = cv2.aruco.DetectorParameters() # updated to work with newer OpenCV
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100) # updated to work with newer OpenCV
    
    def detect_marker_positions(self, img):
        # Perform detection
        corners, ids, rejected = cv2.aruco.detectMarkers(
            img, self.aruco_dict, parameters=self.aruco_params)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.distortion_params)
        # rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.distortion_params) # use this instead if you got a value error
        
        if ids is None:
            return [], img

        # Compute the marker positions
        measurements = []
        seen_ids = []
        for i in range(len(ids)):
            idi = ids[i,0] #idi is the tag of the marker
            # Some markers appear multiple times but should only be handled once.
            if idi in seen_ids:
                continue
            else:
                seen_ids.append(idi)

            lm_tvecs = tvecs[ids == idi]
            lm_rvecs = rvecs[ids == idi]
            #finding the rotation matrix of the marker
            # print(f"lm_rvecs is \n{lm_rvecs}")
            
            
            
            if(len(lm_tvecs) > 1):
                angle = max(lm_rvecs[0][2], lm_rvecs[1][2])
            else:
                angle = lm_rvecs[0][2]
                
                
            xChange = -(self.marker_length/2) * np.cos(angle)
            yChange = (self.marker_length/2) * np.sin(angle)

            
            for i in range(len(lm_tvecs)):
                #in the case if we mutiple face in the same time 
                # print(f"the length of t vec is {len(lm_tvecs)}")
                # print(f"translation vector is {lm_tvecs[i]}")
                # print(f"rotation vect is {lm_rvecs[i]}")
                # print("here")
                # R,_ = cv2.Rodrigues(lm_rvecs[i])  #finding the roation matrix from rvec 
                LMcenter = np.array([[self.marker_length/2,0,0]]).T
                # rotationChange = np.array(np.dot(R,LMcenter).T) #compute the change caused by rotation of the marker in the camera's frame
                
                #extract the centre of the block by 
                # xChange = rotationChange[0][1]
                # yChange = rotationChange[0][0]
                

                xChange = -(self.marker_length/2) * np.cos(lm_rvecs[i][2])
                yChange = (self.marker_length/2) * np.sin(lm_rvecs[i][2])
                
                # xChange = 0
                # yChange = 0
                # print(f"the roation change matrix is {rotationChange}")
            #     print(f"x change is {xChange} and y change is {yChange}")
            # print(f"x change is {xChange} and y change is {yChange}")
            #unrotate the marker                 
            lm_tvecs = lm_tvecs.T
            
            
            lm_bff2d = np.block([[lm_tvecs[2,:]-xChange],[-lm_tvecs[0,:]-yChange]]) #the relative position of the marker w.r.t camera position
            # lm_bff2d = np.block([[lm_tvecs[2,:]],[-lm_tvecs[0,:]]])
            lm_bff2d = np.mean(lm_bff2d, axis=1).reshape(-1,1)
            
            # lm_bff2d = [[lm_tvecs[2,:]-xChange],[-lm_tvecs[0,:]-yChange]]
            
            print(f"\n\n========marker {idi} is dectected========")
            # print(f"\t tvec is \n{tvecs}\n\n")
            # print(f"\t rvec is \n{rvecs}\n")
            print(f"\t lm buff is \n{lm_tvecs[2,:]},{-lm_tvecs[0,:]} \nvs\n {lm_bff2d}\n")

            
            

            lm_measurement = measure.Marker(lm_bff2d, idi)
            measurements.append(lm_measurement)
        
        # Draw markers on image copy
        img_marked = img.copy()
        # cv2.drawFrameAxes(img_marked,self.camera_matrix,self.distortion_params,rvecs,tvecs,self.marker_length)
        cv2.aruco.drawDetectedMarkers(img_marked, corners, ids)

        return measurements, img_marked
