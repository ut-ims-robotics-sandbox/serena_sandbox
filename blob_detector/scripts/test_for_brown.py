#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np

low_hue = 14
low_sat = 175
low_val = 99
high_hue = 19
high_sat = 255
high_val = 145


x_coord = 400
blob_size = 120 


new_img_available = False


state_tracking = True
state_approaching_blob = False


color = 1


def get_image(image_msg):

    global new_img_available, new_img_msg

    new_img_available = True
    new_img_msg = image_msg



def main():
    global velocity_pub, bridge, blob_parameters, detector, color, new_img_available, new_img_msg, low_hue, low_sat, low_val, high_hue, high_sat, high_val, cv_image, hsv_image, lower_limits, upper_limits, thresholded_image, keypoints, first_blob, robot_vel, x_coord, y_coord, blob_size, state_tracking, state_approaching_blob, new_blob_available


    rospy.init_node("follow_three_blobs_final")


    rospy.Subscriber("camera/color/image_raw", Image, get_image)

    velocity_pub = rospy.Publisher("robotont/cmd_vel", Twist, queue_size=1)


    bridge = cv_bridge.core.CvBridge()

    cv2.namedWindow("Thresholded image", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('blob_detector/follow_three_blobs_final')



    blob_parameters = cv2.SimpleBlobDetector_Params()


    blob_parameters.filterByColor = True
    blob_parameters.filterByArea = False
    blob_parameters.filterByCircularity = False
    blob_parameters.filterByInertia = False
    blob_parameters.filterByConvexity = False


    blob_parameters.blobColor = 255 

    blob_parameters.minArea = 1491
    blob_parameters.maxArea = 307200

    blob_parameters.minCircularity = 0
    blob_parameters.maxCircularity = 1

    blob_parameters.minInertiaRatio = 0
    blob_parameters.maxInertiaRatio = 1

    blob_parameters.minConvexity = 0
    blob_parameters.maxConvexity = 1


    blob_parameters.minDistBetweenBlobs = 100


    detector = cv2.SimpleBlobDetector_create(blob_parameters)


    while not rospy.is_shutdown():




        if new_img_available:

            cv_image = bridge.imgmsg_to_cv2(new_img_msg, desired_encoding='bgr8')

            cv2.imshow('blob_detector/follow_three_blobs_final', cv_image)

            cv2.waitKey(1)

            new_img_available = False
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


            lower_limits = np.array([low_hue, low_sat, low_val])
            upper_limits = np.array([high_hue, high_sat, high_val])


            thresholded_image = cv2.inRange(hsv_image, lower_limits, upper_limits)
            thresholded_image = cv2.bitwise_and(cv_image, cv_image, mask = thresholded_image)


            cv2.imshow("Thresholded image", thresholded_image)

            keypoints = detector.detect(thresholded_image)


            if len(keypoints) > 0:
                first_blob = keypoints[0]
                x_coord = int(first_blob.pt[0])
                y_coord = int(first_blob.pt[1])
                blob_size = int(first_blob.size)
                print(x_coord, y_coord, blob_size)






        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


cv2.destroyAllWindows()



if __name__ == '__main__':
    main()

 
