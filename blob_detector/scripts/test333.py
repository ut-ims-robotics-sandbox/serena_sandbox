#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Point
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np



def get_image(image_msg):
    # This is the callback function that gets called whenever
    # a new Image type message is published on the usb_cam/image_raw topic.
    
    global new_img_available, new_img_msg

    new_img_available = True
    new_img_msg = image_msg



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





def update_color(new_value):
    global color
    color = new_value

def update_min_area(new_value):
    global min_area
    min_area = new_value

def update_max_area(new_value):
    global max_area
    max_area = new_value

def update_min_circularity(new_value):
    global min_circularity
    min_circularity = new_value

def update_max_circularity(new_value):
    global max_circularity
    max_circularity = new_value

def update_min_inertia_ratio(new_value):
    global min_inertia_ratio
    min_inertia_ratio = new_value

def update_max_inertia_ratio(new_value):
    global max_inertia_ratio
    max_inertia_ratio = new_value

def update_min_convexity(new_value):
    global min_convexity
    min_convexity = new_value

def update_max_convexity(new_value):
    global max_convexity
    max_convexity = new_value

def update_min_dist_between_blobs(new_value):
    global min_dist_between_blobs
    min_dist_between_blobs = new_value




# Create a boolean to keep track of whether we have a new image available or not
new_img_available = False


# Initialise the global variables that contain the threshold values
# so that we can later change them by moving the sliders
low_hue = 38
low_sat = 59
low_val = 199
high_hue = 54
high_sat = 255
high_val = 239


color = 255
min_area = 0
max_area = 307200
min_circularity = 0
max_circularity = 100
min_inertia_ratio = 0
max_inertia_ratio = 100
min_convexity = 0
max_convexity = 100
min_dist_between_blobs = 100


    
def main():
    global new_img_available, low_hue, low_sat, low_val, high_hue, high_sat, high_val, color, min_area, max_area, min_circularity, max_circularity, min_inertia_ratio, max_inertia_ratio, min_convexity, max_convexity, min_dist_between_blobs
 
   
    rospy.init_node("go_to_green_square")
    
    # Create a subscriber that starts listening for incoming messages
    # of the type Image on the usb_cam/image_raw topic
    rospy.Subscriber("usb_cam/image_raw", Image, get_image)
    blob_position_pub = rospy.Publisher("blob_location", Point, queue_size=1)
    

    bridge = cv_bridge.core.CvBridge()

    cv2.namedWindow("Thresholded image", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('blob_detector/go_to_green_square')
    cv2.namedWindow("Blob parameters", cv2.WINDOW_AUTOSIZE)

 # Here we create the sliders that regulate the range of colour we wish to filter
    cv2.createTrackbar("low_hue", "Thresholded image", low_hue, 179, update_low_hue)
    cv2.createTrackbar("high_hue", "Thresholded image", high_hue, 179, update_high_hue)
    cv2.createTrackbar("low_sat", "Thresholded image", low_sat, 255, update_low_sat)
    cv2.createTrackbar("high_sat", "Thresholded image", high_sat, 255, update_high_sat)
    cv2.createTrackbar("low_val", "Thresholded image", low_val, 255, update_low_val)
    cv2.createTrackbar("high_val", "Thresholded image", high_val, 255, update_high_val)

    
    cv2.createTrackbar("color", "Blob parameters", color, 255, update_color)
    cv2.createTrackbar("min_area", "Blob parameters", min_area, 307200, update_min_area)
    cv2.createTrackbar("max_area", "Blob parameters", max_area, 307200, update_max_area)
    cv2.createTrackbar("min_circularity", "Blob parameters", min_circularity, 100, update_min_circularity)
    cv2.createTrackbar("max_circularity", "Blob parameters", max_circularity, 100, update_max_circularity)
    cv2.createTrackbar("min_inertia_ratio", "Blob parameters", min_inertia_ratio, 100, update_min_inertia_ratio)
    cv2.createTrackbar("max_inertia_ratio", "Blob parameters", max_inertia_ratio, 100, update_max_inertia_ratio)
    cv2.createTrackbar("min_convexity", "Blob parameters", min_convexity, 100, update_min_convexity)
    cv2.createTrackbar("max_convexity", "Blob parameters", max_convexity, 100, update_max_convexity)
    cv2.createTrackbar("min_dist_between_blobs", "Blob parameters", min_dist_between_blobs, 640, update_min_dist_between_blobs)



    blob_parameters = cv2.SimpleBlobDetector_Params()

# Set all filters either on or off
    blob_parameters.filterByColor = True
    blob_parameters.filterByArea = True
    blob_parameters.filterByCircularity = True
    blob_parameters.filterByInertia = True
    blob_parameters.filterByConvexity = True


    while not rospy.is_shutdown():
        # Only show a fresh image when we have one from the camera
        # This is a way of optimising performance
        if new_img_available:
            # Convert the image from Image message to OpenCV image format
            cv_image = bridge.imgmsg_to_cv2(new_img_msg, desired_encoding='bgr8')
           
            cv2.imshow('blob_detector/go_to_green_square', cv_image)
            # Show the image for 1 ms. Without this line, the program will not work.
            cv2.waitKey(1)
            # Set the boolean to False, indicating that we have used up the camera image
            new_img_available = False
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            

# Create arrays from lower and upper hue, saturation and value limits
    # because the thresholding function cv2.inRange() takes numpy arrays as input
    
            lower_limits = np.array([low_hue, low_sat, low_val])
            upper_limits = np.array([high_hue, high_sat, high_val])

    # Threshold the image using the defined value ranges that are changable with sliders
            thresholded_image = cv2.inRange(hsv_image, lower_limits, upper_limits)
            thresholded_image = cv2.bitwise_and(cv_image, cv_image, mask = thresholded_image)

    # Show the thresholded image in the same frame where we placed the sliders
            cv2.imshow("Thresholded image", thresholded_image)

    
            blob_parameters.blobColor = color # Detect either black or white objects

# Filter by size
            blob_parameters.minArea = min_area
            blob_parameters.maxArea = max_area

# Filter by shape
            blob_parameters.minCircularity = min_circularity/100
            blob_parameters.maxCircularity = max_circularity/100

            blob_parameters.minInertiaRatio = min_inertia_ratio/100
            blob_parameters.maxInertiaRatio = max_inertia_ratio/100

            blob_parameters.minConvexity = min_convexity/100
            blob_parameters.maxConvexity = max_convexity/100

# Set the minimum distance that needs to be between two blobs
# in order for them both to be detected
            blob_parameters.minDistBetweenBlobs = min_dist_between_blobs

# Define the blob detector
            detector = cv2.SimpleBlobDetector_create(blob_parameters)

    # Use the blob detector to detect shapes on the thresholded image
            keypoints = detector.detect(thresholded_image)

    # Only attempt to access attributes of the detected shapes if any are found
            if len(keypoints) > 0:
                first_blob = keypoints[0]
               
                x_coord = int(first_blob.pt[0])
                y_coord = int(first_blob.pt[1])
                blob_size = int(first_blob.size)

                print(x_coord, y_coord, blob_size)

                blob_position = Point()
                blob_position.x = x_coord
                blob_position.y = y_coord
          
                blob_position_pub.publish(blob_position)
            

    # If the letter Q is pressed on the keyboard, end the while-loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Make sure no OpenCV processes are left running and all related windows are closed
cv2.destroyAllWindows()




if __name__ == '__main__':
    main()

   
