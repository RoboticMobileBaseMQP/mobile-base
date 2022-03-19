#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from mobile_base_control.msg import task
import time
import sys, tty, select, termios


class TaskCreator:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("task_creator", anonymous=True)
      
        self.task_publisher = rospy.Publisher('/task_topic', task, queue_size=10)

    def send_task(self, type, base, elevator, arm, gripper):
        t = task()
        t.task_type = type
        t.base_orientation = base
        t.elevator_height = elevator
        if type == t.TASK_PICK:
            t.arm_orientation = arm
        else:
            t.arm_joint_angles = arm
        t.gripper_closed = gripper

        self.task_publisher.publish(t)
        print("published")

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    m = TaskCreator(init_node=True)

    t = task()
    base_orientation = PoseStamped()
    base_orientation.pose.position.x = 1
    elevator_height = 0.0
    arm_orientation = PoseStamped()
    arm_joint_angles = [0.0]*7
    gripper_closed = False

    settings = termios.tcgetattr(sys.stdin)


    while not rospy.is_shutdown():
        x = getKey(None)

        if x not in ["1", "2", "3"]:
            print("bad input, try again")
            # continue

        if x == "1":
            #TODO: fill in test values
            print("sending pick task")

            m.send_task(
                t.TASK_PICK,
                base_orientation,
                elevator_height,
                arm_orientation,
                gripper_closed
            )
        
        if x == "2":
            #TODO: fill in test values
            print("sending home task")


            m.send_task(
                t.TASK_HOME,
                base_orientation,
                elevator_height,
                arm_joint_angles,
                gripper_closed
            )
            
        if x == "3":
            #TODO: fill in test values
            print("sending swap task")

            m.send_task(
                t.TASK_SWAP,
                base_orientation,
                elevator_height,
                arm_joint_angles,
                gripper_closed
            )
        
        if (x == '\x03'):
            break
