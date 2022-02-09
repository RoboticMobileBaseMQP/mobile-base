#!/usr/bin/env python3

from distutils.sysconfig import get_config_var
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from abstract_arm.srv import moveToPose, moveToPoseResponse

from std_msgs.msg import Bool

class MoveItPlanner:
    def __init__(self) -> None:
        rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        
        # Initialize nodes and service
        rospy.init_node("move_group_interface", anonymous=True)
        s = rospy.Service('/move_it_planner', moveToPose, self.move_arm)
        
        # launch individual arm groups
        arm_name = rospy.get_param("/arm")
        print(arm_name)

        if arm_name == "panda": 
            print("Initalize MoveIt for panda")

            # Initialize roscpp
            joint_state_topic = ['joint_states:=/panda/joint_states'] # specific to panda. Not necessary with Kortex
            moveit_commander.roscpp_initialize(joint_state_topic)

            self.robot = moveit_commander.RobotCommander()
            self.arm_group_name = "panda_arm" # arm for kortex, panda_arm for panda
            self.scene = moveit_commander.PlanningSceneInterface() # ns=rospy.get_namespace()
            self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name) # ns=rospy.get_namespace()
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        else:
            print("initialize moveit for kortex")
            
            # Initialize roscpp
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node("move_group_interface", anonymous=True)

            self.robot = moveit_commander.RobotCommander('robot_description')
            self.arm_group_name = "arm"
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace()) # ns=rospy.get_namespace()
            self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        
        self.TOLERANCE = 0.01 
               
        # print(rospy.get_namespace())

        # planning_frame = self.arm_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # # We can also print the name of the end-effector link for this group:
        # eef_link = self.arm_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # # We can get a list of all the groups in the robot:
        # group_names = self.robot.get_group_names()
        # print("============ Available Planning Groups:", self.robot.get_group_names())

        # # Sometimes for debugging it is useful to print the entire state of the
        # # robot:
        # print("============ Printing robot state")
        # print(self.robot.get_current_state())
        # print("")

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
