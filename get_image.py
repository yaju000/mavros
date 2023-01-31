#!/usr/bin/env python 
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
from std_msgs.msg import String
import cv2 
import numpy as np

bridge_object = CvBridge() # create the cv_bridge object 

class hsv():
    def __init__(self):
        self.center = [0, 0]
        self.image_received = 0 #Flag to indicate that we have already received an image 
        self.cv_image = 0 

        rospy.Subscriber("/iris_0/usb_cam/image_raw", Image, self.camera_callback) 


    def process_image(self, image):

        image = cv2.resize(image,(480,360)) 
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

        r_lower = np.array([140, 60, 130])
        r_upper = np.array([190, 140, 255])
        r_mask = cv2.inRange(hsv, r_lower, r_upper)
        #kernal = np.ones((11, 11), "uint8")
        kernal = np.ones((25, 25), "uint8")
        r_mask = cv2.dilate(r_mask, kernal)
        target = cv2.bitwise_and(image, image, mask=r_mask)
        r_mask, contours, hierarchy = cv2.findContours(r_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                p = x + (w//2)
                q = y + (h//2)
                self.center = [p, q]
                center = self.center
                Target = 'Target:'+str(self.center)
                cv2.putText(image, Target, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
                        #self.center = rospy.center()
                
                #if (q>80):
                    #print('Target', self.center)
                    #print("Number of Contours found = " + str(len(contours)))
                
            cv2.imshow("detection", image)
            cv2.waitKey(1)

        # min_green = np.array([50,220,220]) 
        # max_green = np.array([60,255,255]) 
        # min_red = np.array([0,0,225]) 
        # max_red = np.array([125,125,255]) 
        # min_blue = np.array([110,220,220]) 
        # max_blue = np.array([120,255,255]) 
        
        # mask_g = cv2.inRange(hsv, min_green, max_green) 
        # mask_r = cv2.inRange(hsv, min_red, max_red) 
        # mask_b = cv2.inRange(hsv, min_blue, max_blue) 

        # res_b = cv2.bitwise_and(image, image, mask= mask_b) 
        # res_g = cv2.bitwise_and(image,image, mask= mask_g) 
        # res_r = cv2.bitwise_and(image,image, mask= mask_r) 
        # #cv2.imshow('Green',res_g) 
        # cv2.imshow('Red',res_b) 
        # #cv2.imshow('Blue',res_r) 
        # cv2.imshow('Original',image) 
        # cv2.waitKey(1) 
        

    def camera_callback(self, data): 
 
        self.image_received=1 
        try: 
            #print("received ROS image, I will convert it to opencv") 
            # We select bgr8 because its the OpenCV encoding by default 
            self.cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 
            #Add your code to save the image here: 
            #Save the image "img" in the current path  
            #cv2.imwrite('uav_image.jpg', self.cv_image)        
            ## Calling the processing function
            self.process_image(self.cv_image)
            #cv2.imshow('Image from uav camera', self.cv_image) 

        except CvBridgeError as e: 
            print(e) 

    def value_callback(self):
        center_position = self.center
        return center_position

    def iteration(self,event):
        print('center = ')
        print(self.center)
        

if __name__ == '__main__': 
    rospy.init_node('hsv', anonymous=True) 
     
    dt = 1.0/20
    pathplan_run = hsv()
    rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
    rospy.spin()
