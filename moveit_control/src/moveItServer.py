#!/usr/bin/env python3

from distutils.sysconfig import get_config_var
import sys
import rospy
import moveit_commander
import moveit_msgs.msg

from moveit_control.srv import moveToPose,moveToPoseResponse
from std_msgs.msg import Bool

class MoveItPlanner:
    def __init__(self) -> None:

        rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=True)
        
        s = rospy.Service('/move_it_planner', moveToPose, self.move_arm)

        self.robot = moveit_commander.RobotCommander("robot_description")
        self.arm_group_name = "arm"
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns=rospy.get_namespace())
        selfdisplay_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
        
        self.TOLERANCE = 0.01
        
        rospy.loginfo("Ready to accept poses!")

    def get_cartesian_pose(self, arm_group):

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def move_arm(self, pose):
        bool = Bool()

        try:
            previous_pose = self.arm_group.get_current_pose()
            previous_pose = previous_pose.pose
            
            self.arm_group.set_goal_position_tolerance(self.TOLERANCE)
            self.arm_group.set_pose_target(pose.pose)
            rospy.loginfo("Planning and going to waypoint")
            self.arm_group.go(wait=True)
            self.get_cartesian_pose(self.arm_group)

            bool.data = True
            print("Success")
            return moveToPoseResponse(bool)

        except Exception as e:
            print(e)

            bool.data = False
            return moveToPoseResponse(bool)




if __name__ == "__main__":
    moveitplanner = MoveItPlanner()
    rospy.spin()
