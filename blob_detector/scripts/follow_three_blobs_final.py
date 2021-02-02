#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int32
from blob_detector.msg import blob
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np


low_hue = 120
low_sat = 254
low_val = 102
high_hue = 121
high_sat = 255
high_val = 103


x_coord = 400
size = 120 


new_img_available = False
new_blob_available = False


state_tracking = True

state_approaching_blob = False

color = 1



def get_image(image_msg):

    global new_img_available, new_img_msg

    new_img_available = True
    new_img_msg = image_msg


def get_blob(blob_position):
    global new_blob_available, x_coord, size
    new_blob_available = True
    x_coord = blob_position.position.x
    size = blob_position.size       




    #global robot_vel, velocity_pub, low_hue, low_sat, low_val, high_hue, high_sat, high_val, lower_limits,                 upper_limits,                     thresholded_image, hsv_image, cv_image, keypoints, first_blob, x_coord_, y_coord_, blob_size,         blob_position, state_found_blob






def main():
    global blob_position_pub, velocity_pub, bridge, blob_parameters, detector, new_img_available, low_hue, low_sat, low_val, high_hue, high_sat, high_val, cv_image, hsv_image, state_tracking, state_approaching_blob, new_blob_available, robot_vel, x_coord, size, color


    rospy.init_node("follow_three_blobs_final")


    rospy.Subscriber("camera/color/image_raw", Image, get_image)
    blob_position_pub = rospy.Publisher("blob_location", blob, queue_size=1)

    rospy.Subscriber("blob_location", blob, get_blob)
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

        if color == 1 :
            low_hue = 120
            low_sat = 254
            low_val = 102
            high_hue = 121
            high_sat = 255
            high_val = 103

        elif color == 2 :
            low_hue = 1
            low_sat = 2
            low_val = 1
            high_hue = 1
            high_sat = 2
            high_val = 1

        else :
            low_hue = 0
            low_sat = 4
            low_val = 12
            high_hue = 11
            high_sat = 25
            high_val = 10

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
                x_coord_ = int(first_blob.pt[0])
                y_coord_ = int(first_blob.pt[1])
                blob_size = int(first_blob.size)
                print(x_coord_, y_coord_, blob_size)
                blob_position = blob()
                blob_position.position.x = x_coord_
                blob_position.position.y = y_coord_
                blob_position.size = blob_size
                blob_position_pub.publish(blob_position)
                state_tracking = False
                state_approaching_blob = True

        if state_approaching_blob :

            if new_blob_available :
                robot_vel = Twist()
                if x_coord < 400 :           
                    robot_vel.angular.z = 0.15
                elif x_coord > 400 :          
                    robot_vel.angular.z = -0.15          
                else:
                    robot_vel.angular.z = 0.0

                if size < 120:
                    robot_vel.linear.x = 0.1 
                elif size > 120:
                    robot_vel.linear.x = -0.1
                else:
                    robot_vel.linear.x = 0.0

                velocity_pub.publish(robot_vel)

                new_blob_available = False

            if size == 120 and x_coord == 400 :

                robot_vel.linear.x = 0.0
                robot_vel.linear.y = 0.0
                robot_vel.angular.z = 0.0

                velocity_pub.publish(robot_vel)
                state_approaching_blob = False
                state_tracking = True

                color = color + 1



        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


cv2.destroyAllWindows()



if __name__ == '__main__':
    main()

 
