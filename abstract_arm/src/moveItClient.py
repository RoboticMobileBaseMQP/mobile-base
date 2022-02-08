#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from abstract_arm.srv import moveToPose
from std_msgs.msg import Bool
import tf2_ros

class MoveItClient:
    def __init__(self) -> None:
        
        rospy.init_node("moveit_client", anonymous=True)

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)

    
    def move_arm_client(self, pose):
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
    moveItClient = MoveItClient()
    
    try:
        # Find relationship between wrold and base of robot arm
        arm_base_name = 'panda_link0' #specific to panda. Need to modularize
        world2Base = moveItClient.tfBuffer.lookup_transform('world', arm_base_name, rospy.Time())

        # Random poses inside of the robot arm workspace
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.5
        pose_goal.position.x = 0.5
        pose_goal.position.y = -0.5
        pose_goal.position.z = 0.5

        pose_goal.position.x += world2Base.transform.translation.x
        pose_goal.position.y += world2Base.transform.translation.y
        pose_goal.position.z += world2Base.transform.translation.z
        
        print("Requesting.. ")
        print(pose_goal)
        
        moveItClient.move_arm_client(pose_goal)

        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = -0.2
        # pose_goal.position.z = 0.4
        # print("Requesting.. ")
        # move_arm_client(pose_goal)

        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = -0.4
        # pose_goal.position.y = -0.2
        # pose_goal.position.z = 0.4
        # print("Requesting.. ")
        # move_arm_client(pose_goal)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("TF TRANSFORM ERROR!")
        pass
        