#!/usr/bin/env python3

import rospy
from pathlib import Path 

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 
import numpy as np
# Initialisations: 
node_name = "object_detection_node"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(5)

base_image_path = Path.home().joinpath("myrosdata/task4_images/")
base_image_path.mkdir(parents=True, exist_ok=True) 

cvbridge_interface = CvBridge() 

waiting_for_image = True 

def show_and_save_image(img, img_name): 
    full_image_path = base_image_path.joinpath(f"{img_name}.jpg") 

    print("Opening the image in a new window...")
    cv2.imshow(img_name, img) 
    print(f"Saving the image to '{full_image_path}'...")
    cv2.imwrite(str(full_image_path), img) 
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes") 
    print("Please close down the image pop-up window to continue...")
    cv2.waitKey(0) 

def camera_cb(img_data): 
    global waiting_for_image 
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8") 
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True: 
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        colors = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'blue': [(100, 150, 0), (140, 255, 255)],
            'green': [(35, 100, 50), (85, 255, 255)],
            'turquoise': [(60,210,200), (70, 230, 220)]
        }

        dominant_color = None
        max_count = 0

        for color, (lower, upper) in colors.items():
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv_img, lower, upper)
            count = cv2.countNonZero(mask)

            if count > max_count:
                max_count = count
                dominant_color = color

        if dominant_color:
            print(f"The dominant color in the initial zone is {dominant_color}")
        else:
            print("No dominant color detected")

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb) 

while waiting_for_image: 
    rate.sleep()

cv2.destroyAllWindows() 
