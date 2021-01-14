#!/usr/bin/env python2
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
import numpy as np

# Create the bridge that allows us to take Image type messages and
# convert them to OpenCV image format
bridge = cv_bridge.core.CvBridge()

# Create a boolean to keep track of whether we have a new image available or not
new_img_available = False

def get_image(image_msg):
    # This is the callback function that gets called whenever
    # a new Image type message is published on the usb_cam/image_raw topic.
    
    global new_img_available, new_img_msg

    new_img_available = True
    new_img_msg = image_msg
    
def main():
    global new_img_available
    
    # Initialise a node called "camera_listener".
    # Add the anonymous=True flag to ensure our node gets a unique name
    # even if some other node named "camera_listener" is already running.
    
    rospy.init_node('imgmsg_to_cv2img', anonymous=True)
    
    # Create a subscriber that starts listening for incoming messages
    # of the type Image on the usb_cam/image_raw topic
    rospy.Subscriber('usb_cam/image_raw', Image, get_image)

    # Create a window where we will show the camera feed
    cv2.namedWindow('blob_detector/imgmsg_to_cv2img')

    # This cycle will run as long as rospy still runs.
    # This is very similar to while True
    while not rospy.is_shutdown():
        # Only show a fresh image when we have one from the camera
        # This is a way of optimising performance
        if new_img_available:
            # Convert the image from Image message to OpenCV image format
            cv_image = bridge.imgmsg_to_cv2(new_img_msg, desired_encoding='bgr8')
            # Show the image in the window we created before the while-loop
            cv2.imshow('blob_detector/imgmsg_to_cv2img', cv_image)
            # Show the image for 1 ms. Without this line, the program will not work.
            cv2.waitKey(1)
            # Set the boolean to False, indicating that we have used up the camera image
            new_img_available = False


if __name__ == '__main__':
    main()
