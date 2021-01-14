#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist

def driveAlongX(linx_value, duration):
    for i in range(0,duration):
        vel_msg.linear.x = linx_value
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.1)

def driveAlongY(liny_value, duration):
    for i in range(0,duration):
        vel_msg.linear.x = 0
        vel_msg.linear.y = liny_value
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.1)

def rotateAroundZ(angz_value, duration):
    for i in range(0,duration):
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = angz_value
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.1)

def stop():
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def main():    
    
    while not rospy.is_shutdown():       
        driveAlongX(0.1, 30)
        stop()
        driveAlongY(-0.1, 30)
        stop()
        driveAlongX(-0.1, 30)
        stop()
        driveAlongY(0.1, 30)
        stop()
        loop_rate.sleep()

rospy.init_node("sideways_square_node")
velocity_publisher = rospy.Publisher("robotont/cmd_vel", Twist, queue_size=1)
loop_rate = rospy.Rate(10)
vel_msg = Twist()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
