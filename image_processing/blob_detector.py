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

def update_filter_by_area(new_value):
    global filter_by_area
    filter_by_area = new_value

def update_filter_by_circularity(new_value):
    global filter_by_circularity
    filter_by_circularity = new_value

def update_filter_by_inertia(new_value):
    global filter_by_inertia
    filter_by_inertia = new_value

def update_filter_by_convexity(new_value):
    global filter_by_convexity
    filter_by_convexity = new_value



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


# Read in the greyscale image
orig_image = cv2.imread("../image_processing/tennis.jpeg") # Image source: https://media.defense.gov/2009/Oct/26/2000447368/-1/-1/0/091023-F-5924C-002.JPG

# Create SimpleBlobDetector_Params() object
blob_parameters = cv2.SimpleBlobDetector_Params()



# Initialise the global variables
blob_parameters.filter_by_color = 0
blob_parameters.filter_by_area = 0
blob_parameters.filter_by_circularity = 0
blob_parameters.filter_by_inertia = 0
blob_parameters.filter_by_convexity = 0


blob_parameters.color = 0


blob_parameters.min_area = 0
blob_parameters.max_area = 0

blob_parameters.min_circularity = 0
blob_parameters.max_circularity = 0

blob_parameters.min_inertia_ratio = 0
blob_parameters.max_inertia_ratio = 0

blob_parameters.min_convexity = 0
blob_parameters.max_convexity = 0


blob_parameters.min_dist_between_blobs = 0

 # In order to place sliders in a window, we need to first create this window
cv2.namedWindow("Blob Detector", cv2.WINDOW_AUTOSIZE)

 # Here we create the sliders 
cv2.createTrackbar("filter_by_color", "Blob Detector", filter_by_color, 1, update_filter_by_color)
cv2.createTrackbar("filter_by_area", "Blob Detector", filter_by_area, 1, update_filter_by_area)
cv2.createTrackbar("filter_by_circularity", "Blob Detector", filter_by_circularity, 1, update_filter_by_circularity)
cv2.createTrackbar("filter_by_inertia", "Blob Detector", filter_by_inertia, 1, update_filter_by_inertia)
cv2.createTrackbar("filter_by_convexity", "Blob Detector", filter_by_convexity, 1, update_filter_by_convexity)



cv2.createTrackbar("color", "Blob Detector", color, 255, update_color)

cv2.createTrackbar("min_area", "Blob Detector", min_area, 40000, update_min_area)
cv2.createTrackbar("max_area", "Blob Detector", max_area, 40000, update_max_area)

cv2.createTrackbar("min_circularity", "Blob Detector", min_circularity, 100, update_min_circularity)
cv2.createTrackbar("max_circularity", "Blob Detector", max_circularity, 100, update_max_circularity)

cv2.createTrackbar("min_inertia_ratio", "Blob Detector", min_inertia_ratio, 100, update_min_inertia_ratio)
cv2.createTrackbar("max_inertia_ratio", "Blob Detector", max_inertia_ratio, 100, update_max_inertia_ratio)

cv2.createTrackbar("min_convexity", "Blob Detector", min_convexity, 100, update_min_convexity)
cv2.createTrackbar("max_convexity", "Blob Detector", max_convexity, 100, update_max_convexity)

cv2.createTrackbar("min_dist_between_blobs", "Blob Detector", min_dist_between_blobs, 1, update_min_dist_between_blobs)





# Set all filters either on or off
blob_parameters.filterByColor = True
blob_parameters.filterByArea = True
blob_parameters.filterByCircularity = False
blob_parameters.filterByInertia = False
blob_parameters.filterByConvexity = False

# Define filter ranges. In practical applications, these only need
# to be set for the filters that are turned on
# Filter by colour
blob_parameters.blobColor = {0, 255} # Detect either black or white objects

# Filter by size
blob_parameters.minArea = {some number}
blob_parameters.maxArea = {some number}

# Filter by shape
blob_parameters.minCircularity = {some number}
blob_parameters.maxCircularity = {some number}

blob_parameters.minInertiaRatio = {some number}
blob_parameters.maxInertiaRatio = {some number}

blob_parameters.minConvexity = {some number}
blob_parameters.maxConvexity = {some number}

# Set the minimum distance that needs to be between two blobs
# in order for them both to be detected
blob_parameters.minDistBetweenBlobs = {some number}

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



