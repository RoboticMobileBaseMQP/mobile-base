#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from moveit_control.srv import moveToPose
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def move_arm_client(pose):
    rospy.wait_for_service('/move_it_planner')
    try:
        c = rospy.ServiceProxy('/move_it_planner', moveToPose)
        success = c(pose)
        bool = Bool()
        bool.data = True
        if success.data == bool:
            return
        else:
            return print(str(success.data.data) + ", unable to find a solution.")
    
    except rospy.ServiceException as e:
        print("Service called failed: %s"%e)

if __name__ == "__main__":

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 0.3
    print("Requesting.. ")
    move_arm_client(pose_goal)

    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.4
    print("Requesting.. ")
    move_arm_client(pose_goal)

    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.4
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.4
    print("Requesting.. ")
    move_arm_client(pose_goal)