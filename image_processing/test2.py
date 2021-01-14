#!/usr/bin/env python2

import cv2
import numpy as np



# Read in the greyscale image
orig_image = cv2.imread("../image_processing/green_square_stickers/sticker_and_pen.png") # Image source: https://media.defense.gov/2009/Oct/26/2000447368/-1/-1/0/091023-F-5924C-002.JPG




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

# This loop allows us to keep changing slider positions
while True:
    # Convert the RGB image into HSV colour space for easier processing
    hsv_image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)

    # Create arrays from lower and upper hue, saturation and value limits
    # because the thresholding function cv2.inRange() takes numpy arrays as input
    
 

    # Use the blob detector to detect shapes on the thresholded image
    keypoints = detector.detect(hsv_image)

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





   
