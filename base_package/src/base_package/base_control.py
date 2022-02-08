#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

class BaseControl(object):
    def __init__(self):

        self.base_controllers = []

        base_controller_str = "/base/base_{0}_joint_controller/command"

        self.base_controllers.append(rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('z'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10))

        rospy.init_node("base_control_node", anonymous=True)


if __name__ == '__main__':
    b = BaseControl()

    while True:
        print("moving base forward")
        value = Float64()
        value.data = 1
        b.base_controllers[0].publish(value)
        time.sleep(2)