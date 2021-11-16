#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class Move_demo(object):
    def __init__(self):
        rospy.init_node("base_random_move")
        self.velocity_publisher = rospy.Publisher('/base/cmd_vel', Twist, queue_size=10)

        # self.wheel_1_publisher = rospy.Publisher('/base/wheel_1_joint_controller/command', Float64, queue_size=1)


    def back_and_forth(self):
        vel_msg = Twist()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        forward = True

        while not rospy.is_shutdown():
            print("publishing to base/cmd_vel")
            vel_msg.linear.x = 3 if forward else -3
            forward = not forward
            self.velocity_publisher.publish(vel_msg)
            # self.wheel_1_publisher.publish(10 if forward else -10)
            time.sleep(2)

if __name__ == '__main__':
    try:
        print("running main")
        d = Move_demo()
        d.back_and_forth()
    except rospy.ROSInterruptException:
        print("base movement node error")
        
