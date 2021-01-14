#!/usr/bin/env python2
# Import OpenCV to gain access to image processing tools
import cv2

def update_threshold_value(new_value):
    # This is a callback function that is called every time the slider is moved.
    
    # The function takes as input the new slider position, which is
    # then set as the new value of a global variable.

    global threshold
    threshold = new_value

# Read in the greyscale image
gradient_image = cv2.imread("../image_processing/gradient.png")

 # In order to place a slider in a window, we need to first create this window
cv2.namedWindow("Thresholded image", cv2.WINDOW_AUTOSIZE)

# Initialise the global variable that contains the threshold value
threshold = 127 

 # Here we create the slider that regulates what value our threshold is at
cv2.createTrackbar("threshold value", "Thresholded image", threshold, 255, update_threshold_value)


# This loop allows us to keep changing the slider position
while True:
    # Threshold the grescale image in such a way that
    # every value higher than 'threshold' is set to 255 (white)
    _, thresholded_image = cv2.threshold(gradient_image, threshold, 255, cv2.THRESH_BINARY)

    # Show the thresholded image in the same frame where we placed the slider
    cv2.imshow("Thresholded image", thresholded_image)

    # If the letter Q is pressed on the keyboard, end the while-loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Make sure no OpenCV processes are left running and all related windows are closed
cv2.destroyAllWindows()
