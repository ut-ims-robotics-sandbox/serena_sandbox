#!/usr/bin/env python2

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np

# These are callback functions that are called
# every time the corresponding slider is moved.

# The functions take as input the new slider position, which is
# then set as the new value of a global variable.

# Using lambda expressions, it is possible to have just one
# function (instead of six) that updates the correct global variables,
# but that will not be covered in this tutorial.

def update_low_hue(new_value):
    global low_hue
    low_hue = new_value

def update_high_hue(new_value):
    global high_hue
    high_hue = new_value

def update_low_sat(new_value):
    global low_sat
    low_sat = new_value

def update_high_sat(new_value):
    global high_sat
    high_sat = new_value

def update_low_val(new_value):
    global low_val
    low_val = new_value

def update_high_val(new_value):
    global high_val
    high_val = new_value



def get_image(image_msg):
    # This is the callback function that gets called whenever
    # a new Image type message is published on the usb_cam/image_raw topic.
    
    global new_img_available, new_img_msg

    new_img_available = True
    new_img_msg = image_msg


# Read in the greyscale image
orig_image = cv2.imread("../image_processing/green_square_stickers/sticker_and_pen.png") # Image source: https://media.defense.gov/2009/Oct/26/2000447368/-1/-1/0/091023-F-5924C-002.JPG

# Create the bridge that allows us to take Image type messages and
# convert them to OpenCV image format
bridge = cv_bridge.core.CvBridge()

# Create a boolean to keep track of whether we have a new image available or not
new_img_available = False



# Initialise the global variables that contain the threshold values
# so that we can later change them by moving the sliders
low_hue = 0
low_sat = 0
low_val = 0
high_hue = 179
high_sat = 255
high_val = 255

 # In order to place sliders in a window, we need to first create this window
cv2.namedWindow("Thresholded image", cv2.WINDOW_AUTOSIZE)
 # Here we create the sliders that regulate the range of colour we wish to filter
cv2.createTrackbar("low_hue", "Thresholded image", low_hue, 179, update_low_hue)
cv2.createTrackbar("high_hue", "Thresholded image", high_hue, 179, update_high_hue)
cv2.createTrackbar("low_sat", "Thresholded image", low_sat, 255, update_low_sat)
cv2.createTrackbar("high_sat", "Thresholded image", high_sat, 255, update_high_sat)
cv2.createTrackbar("low_val", "Thresholded image", low_val, 255, update_low_val)
cv2.createTrackbar("high_val", "Thresholded image", high_val, 255, update_high_val)


# Create SimpleBlobDetector_Params() object
blob_parameters = cv2.SimpleBlobDetector_Params()

# Set all filters either on or off
blob_parameters.filterByColor = True
blob_parameters.filterByArea = False
blob_parameters.filterByCircularity = False
blob_parameters.filterByInertia = False
blob_parameters.filterByConvexity = False

# Define filter ranges. In practical applications, these only need
# to be set for the filters that are turned on
# Filter by colour
blob_parameters.blobColor = 255 # Detect either black or white objects

# Filter by size
blob_parameters.minArea = 0
blob_parameters.maxArea = 0

# Filter by shape
blob_parameters.minCircularity = 0
blob_parameters.maxCircularity = 0

blob_parameters.minInertiaRatio = 0
blob_parameters.maxInertiaRatio = 0

blob_parameters.minConvexity = 0
blob_parameters.maxConvexity = 0

# Set the minimum distance that needs to be between two blobs
# in order for them both to be detected
blob_parameters.minDistBetweenBlobs = 100

# Define the blob detector
detector = cv2.SimpleBlobDetector_create(blob_parameters)


    
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

# This loop allows us to keep changing slider positions
    while True:
    # Convert the RGB image into HSV colour space for easier processing
        hsv_image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)

    # Create arrays from lower and upper hue, saturation and value limits
    # because the thresholding function cv2.inRange() takes numpy arrays as input
    
        lower_limits = np.array([low_hue, low_sat, low_val])
        upper_limits = np.array([high_hue, high_sat, high_val])

    # Threshold the image using the defined value ranges that are changable with sliders
        thresholded_image = cv2.inRange(hsv_image, lower_limits, upper_limits)
        thresholded_image = cv2.bitwise_and(orig_image, orig_image, mask = thresholded_image)

    # Show the thresholded image in the same frame where we placed the sliders
        cv2.imshow("Thresholded image", thresholded_image)


    # Use the blob detector to detect shapes on the thresholded image
        keypoints = detector.detect(thresholded_image)

    # Only attempt to access attributes of the detected shapes if any are found
        if len(keypoints) > 0:
            for kp in keypoints:
        # Get the coordinates and size for every blob found
                x_coord = int(kp.pt[0])
                y_coord = int(kp.pt[1])
                blob_size = int(kp.size)

        print(x_coord, y_coord, blob_size)
    # If the letter Q is pressed on the keyboard, end the while-loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Make sure no OpenCV processes are left running and all related windows are closed
cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


   
