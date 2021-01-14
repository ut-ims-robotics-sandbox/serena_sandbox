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



#!/usr/bin/env python
# Create SimpleBlobDetector_Params() object
blob_parameters = cv2.SimpleBlobDetector_Params()

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
