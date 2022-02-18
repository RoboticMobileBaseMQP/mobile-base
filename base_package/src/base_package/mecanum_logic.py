#!/usr/bin/env python3

import rospy
from sensor_msgs import Joy
from std_msgs.msg import Float64
from base_package.msg import mecanum_efforts


class MecanumNode:
    def __init__(self) -> None:

        # Listen to the xbox controller!
        rospy.Subscriber("/base/teleop_input", Joy, self.sendEfforts)

        # TODO
        # Listen to reported encoder values
        rospy.Subscriber("/base/mecanum_encoders", mecanum_efforts, self.updatePosition)

        # Publish calculated efforts to low level controllers
        self.efforts = rospy.Publisher("/base/mecanum_efforts", mecanum_efforts, queue_size=10)

        # Publish updates to Simulation
        base_controller_str = "/base/base_{0}_joint_controller/command"
        self.xBaseController = rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10)
        self.yBaseController = rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10)
        self.zBaseController = rospy.Publisher("/base/base_z_rotation_contrmloller/command", Float64, queue_size=10)


    def sendEfforts(self, msg):
        # helper function to send Effort values to cim motors
        # msg [Joy] 
        # Code inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

        L_JoyX = msg.axes[0]
        L_JoyY = msg.axes[1] # potentially multiply this value by 1.1 to counteract imperfect strafing!
        R__JoyX = msg.axes[2]

        demoninator = max(abs(L_JoyX) + abs(L_JoyY) + abs(R__JoyX), 1)

        # Front of robot
        # 1 3
        # 2 4
        # Rear of Robot

        cim1Effort = Float64((L_JoyY + L_JoyX + R__JoyX) / demoninator * 100)
        cim2Effort = Float64((L_JoyY - L_JoyX + R__JoyX) / demoninator * 100)
        cim3Effort = Float64((L_JoyY - L_JoyX - R__JoyX) / demoninator * 100)
        cim4Effort = Float64((L_JoyY + L_JoyX - R__JoyX) / demoninator * 100)

        self.efforts.publish([cim1Effort, cim2Effort, cim3Effort, cim4Effort])




    def updatePosition(self, msg):
        # TODO
        # Publish rough coordinates of x base y base and z rotation to update in sim!



        return
