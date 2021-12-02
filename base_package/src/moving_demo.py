#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class Move_demo(object):
    def __init__(self):
        rospy.init_node("base_random_move")
        self.rate = rospy.Rate(.5)

        self.base_controllers = []
        self.arm_controllers = []

        base_controller_str = "/base/base_{0}_joint_controller/command"
        arm_controller_str = "/my_gen3/kortex_{0}_joint_controller/command"

        self.base_controllers.append(rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('z'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10))

        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('x'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('y'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher("/my_gen3/kortex_z_rotation_controller/command", Float64, queue_size=10))


    def back_and_forth(self):
        value = 1000
        while not rospy.is_shutdown():
            print("publishing velocity")

            value = 1000 if value==-1000 else -1000

            self.base_controllers[3].publish(value)
            self.arm_controllers[3].publish(value)

            time.sleep(1)

if __name__ == '__main__':
    try:
        print("running main")
        d = Move_demo()
        d.back_and_forth()
    except rospy.ROSInterruptException:
        print("base movement node error")
        
