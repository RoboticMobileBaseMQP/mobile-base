#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray
from base_package.msg import effort_list


class ElevatorNode:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("elevator_logic", anonymous=True)

        # Listen to the xbox controller!
        rospy.Subscriber("/joy", Joy, self.sendEfforts)

        # TODO
        # Listen to reported encoder values
        rospy.Subscriber("/base/elevator_encoders", None, self.updatePosition)

        # Publish calculated efforts to low level controllers
        self.elevator_efforts = rospy.Publisher("/base/elevator_efforts", effort_list, queue_size=10)

        # Publish updates to Simulation
        arm_name = rospy.get_param("arm")
        arm_namespace = "panda" if arm_name=="panda" else "my_gen3"
        arm_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"

        self.zInsertController = rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10)

    def sendEfforts(self, msg):
        # helper function to send Effort values to cim motors
        # msg [Joy] 

        AB_buttons = 100*(msg.buttons[1] + -msg.buttons[0]) # B up A down
        XY_buttons = 100*(msg.buttons[3] + -msg.buttons[2]) # Y up X down
        RL_bumpers = 100*(msg.buttons[5] + -msg.buttons[4]) # RB up LB down

        efforts = effort_list()
        efforts.Efforts = [AB_buttons, XY_buttons, RL_bumpers]

        # add feedback loop to ensure all motors reach the same height
        self.elevator_efforts.publish(efforts)


    def updatePosition(self, msg):
        # TODO
        # Publish rough coordinates of elevator to update in sim!
        pass


if __name__=="__main__":
    e = ElevatorNode(init_node=True)
    rospy.spin()