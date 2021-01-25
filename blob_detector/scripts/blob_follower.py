#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int32
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np


def get_x_coord(blob_position):
    global new_point_available, x_coord
    new_point_available = True
    x_coord = blob_position.x

    
def get_size(blob_size):
    global new_size_available, size
    new_size_available = True
    size = blob_size          

new_point_available = False
new_size_available = False
   
x_coord = 400
size = 120 


def main():
    global new_point_available, new_size_available, x_coord, size, robot_vel, velocity_pub

   
    rospy.init_node("blob_follower")
    
    
    rospy.Subscriber("blob_location", Point, get_x_coord)
    rospy.Subscriber("blob_size", Int32, get_size)
    velocity_pub = rospy.Publisher("robotont/cmd_vel", Twist, queue_size=1)

    
    while not rospy.is_shutdown():
        if new_point_available or new_size_available:
            robot_vel = Twist()
            if x_coord > 400 :
                if size > 120:
                    robot_vel.linear.x = -0.1
                elif size < 120:
                    robot_vel.linear.x = 0.1
                else:
                    robot_vel.linear.x = 0.0
                robot_vel.linear.y = 0.0
                robot_vel.angular.z = -0.1

            elif x_coord < 400 :
                if size > 120:
                    robot_vel.linear.x = -0.1
                elif size < 120:
                    robot_vel.linear.x = 0.1
                else:
                    robot_vel.linear.x = 0.0
                robot_vel.linear.y = 0.0
                robot_vel.angular.z = 0.1

            else :
                if size > 120:
                    robot_vel.linear.x = -0.1
                elif size < 120:
                    robot_vel.linear.x = 0.1
                else:
                    robot_vel.linear.x = 0.0 
                robot_vel.linear.y = 0.0
                robot_vel.angular.z = 0.0
            velocity_pub.publish(robot_vel)

            
            new_point_available = False
            new_size_available = False
            



if __name__ == '__main__':
    main()

   
