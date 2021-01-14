#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("velocity_publisher_node")
    velocity_pub = rospy.Publisher("robotont/cmd_vel", Twist, queue_size=1)
    loop_rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        robot_vel = Twist()
        robot_vel.linear.x = 0.1
        robot_vel.linear.y = 0.0
        robot_vel.linear.z = 0.0
        robot_vel.angular.x = 0.0
        robot_vel.angular.y = 0.0
        robot_vel.angular.z = 0.5
        
        velocity_pub.publish(robot_vel)
        loop_rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
