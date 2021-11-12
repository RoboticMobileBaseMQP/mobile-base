#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
    
def main():
    rospy.init_node("base_random_move")
    velocity_publisher = rospy.Publisher('/base/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    forward = True
    print("finished preparing to publish - ", rospy.is_shutdown())

    while not rospy.is_shutdown():
        print("publishing to base/cmd_vel")
        vel_msg.linear.x = 3 if forward else -3
        forward = not forward
        velocity_publisher.publish(vel_msg)
        time.sleep(4)

if __name__ == '__main__':
    try:
        print("running main")
        main()
    except rospy.ROSInterruptException:
        print("base movement node error")
        
