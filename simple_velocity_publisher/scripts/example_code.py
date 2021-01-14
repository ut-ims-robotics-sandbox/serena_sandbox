def driveFWD3sec():
    for i in range(0,30):
       vel_msg.linear.x = 0.2
       vel_msg.linear.y = 0
       vel_msg.angular.z = 0
       velocity_publisher.publish(vel_msg)
       rospy.sleep(0.1)

def driveAlongX(linx_value, duration):
    for i in range(0,duration):
       vel_msg.linear.x = linx_value
       vel_msg.linear.y = 0
       vel_msg.angular.z = 0
       velocity_publisher.publish(vel_msg)
       rospy.sleep(0.1)

def driveAlongY

def rotateAroundZ

def stop


def main()


    while():
        driveAlongX(0.2)
        stop()
        driveAlongY(...)
        stop()
