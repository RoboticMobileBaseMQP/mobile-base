#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf import TransformListener



class Follower(object):
    def __init__(self):
        rospy.init_node("base_follower")
        self.tf = TransformListener()

        self.arm_controllers = []

        arm_controller_str = "/my_gen3/kortex_{0}_joint_controller/command"

        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('x'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('y'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher("/my_gen3/kortex_z_rotation_controller/command", Float64, queue_size=10))

        rospy.Subscriber("tf/", TFMessage, self.callback)

        rospy.spin()

    def callback(self, data):
        try:
            t = self.tf.getLatestCommonTime("/base_link", "/world")
            position, quaternion = self.tf.lookupTransform("/base_link", "/world", t)
            # print(position, quaternion)
            self.arm_controllers[0].publish(position.x)
            self.arm_controllers[1].publish(position.y)
            self.arm_controllers[3].publish(quaternion.z)
            print("-------")
        except Exception:
            pass
        

if __name__ == '__main__':
    try:
        d = Follower()
    except rospy.ROSInterruptException:
        print("Error following base")
        
