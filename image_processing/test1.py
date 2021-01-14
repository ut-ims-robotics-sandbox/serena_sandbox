#!/usr/bin/env python2

import cv2
import numpy as np

# These are callback functions that are called
# every time the corresponding slider is moved.

# The functions take as input the new slider position, which is
# then set as the new value of a global variable.

def update_filter_by_color(new_value):
    global filter_by_color
    filter_by_color = new_value





def update_color(new_value):
    global color
    color = new_value



filter_by_color = 0
color = 0

# Read in the greyscale image
orig_image = cv2.imread("../image_processing/tennis.jpeg") # Image source: https://media.defense.gov/2009/Oct/26/2000447368/-1/-1/0/091023-F-5924C-002.JPG

# Create SimpleBlobDetector_Params() object
blob_parameters = cv2.SimpleBlobDetector_Params()



# Initialise the global variables
blob_parameters.filterByColor = filter_by_color
blob_parameters.filterByArea = 0
blob_parameters.filterByCircularity = 0
blob_parameters.filterByInertia = 0
blob_parameters.filterByConvexity = 0


blob_parameters.blobColor = color # Detect either black or white objects

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
blob_parameters.minDistBetweenBlobs = 0

 # In order to place sliders in a window, we need to first create this window
cv2.namedWindow("Blob Detector", cv2.WINDOW_AUTOSIZE)

 # Here we create the sliders 
cv2.createTrackbar("filter_by_color", "Blob Detector", filter_by_color, 1, update_filter_by_color)
cv2.createTrackbar("color", "Blob Detector", color, 255, update_color)





# Define the blob detector
detector = cv2.SimpleBlobDetector_create(blob_parameters)

# This loop allows us to keep changing the slider position
while True:
    # Threshold the grescale image in such a way that
    # every value higher than 'threshold' is set to 255 (white)
    _, thresholded_image = cv2.threshold(orig_image, color, 255, cv2.THRESH_BINARY)

    # Show the thresholded image in the same frame where we placed the slider
    cv2.imshow("Blob Detector", thresholded_image)

    # If the letter Q is pressed on the keyboard, end the while-loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Make sure no OpenCV processes are left running and all related windows are closed
cv2.destroyAllWindows()



