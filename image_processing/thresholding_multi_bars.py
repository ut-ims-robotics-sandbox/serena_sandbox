#!/usr/bin/env python2

import cv2
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

# Read in the greyscale image
orig_image = cv2.imread("../image_processing/green_square_stickers/sticker_and_pen.png") # Image source: https://media.defense.gov/2009/Oct/26/2000447368/-1/-1/0/091023-F-5924C-002.JPG

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

    # If the letter Q is pressed on the keyboard, end the while-loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Make sure no OpenCV processes are left running and all related windows are closed
cv2.destroyAllWindows()

