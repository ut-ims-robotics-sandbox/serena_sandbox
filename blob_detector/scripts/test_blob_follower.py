#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Point, Twist
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np


def get_x_coord(blob_position):
    global new_point_available, x_coord
    new_point_available = True
    x_coord = blob_position.x



def point_to_twist(x_coord):
    global robot_vel
    robot_vel = Twist()
    if x_coord > 400 :
        robot_vel.linear.x = 0.0
        robot_vel.linear.y = 0.0
        robot_vel.angular.z = -0.5
    elif x_coord < 400 :
        robot_vel.linear.x = 0.0
        robot_vel.linear.y = 0.0
        robot_vel.angular.z = 0.5
    else : 
        robot_vel.linear.x = 0.0
        robot_vel.linear.y = 0.0
        robot_vel.angular.z = 0.0
    velocity_pub.publish(robot_vel)


def stop():
    global robot_vel
    robot_vel = Twist()
    robot_vel.linear.x = 0.0
    robot_vel.linear.y = 0.0
    robot_vel.angular.z = 0.0
    velocity_pub.publish(robot_vel) 
           

new_point_available = False
   
x_coord = 400 


def main():
    global new_point_available, x_coord, robot_vel, velocity_pub

   
    rospy.init_node("blob_follower")
    
    
    rospy.Subscriber("blob_location", Point, get_x_coord)
    velocity_pub = rospy.Publisher("robotont/cmd_vel", Twist, queue_size=1)

    
    while not rospy.is_shutdown():
        if new_point_available:

            
        
            point_to_twist(x_coord)
            
            
            print(x_coord)
            new_point_available = False
        else:
            stop()
            



if __name__ == '__main__':
    main()

   
