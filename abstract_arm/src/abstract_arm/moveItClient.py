#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from abstract_arm.srv import moveToPose
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros

class MoveitArmClient:
    def __init__(self, init_node=False) -> None:

        if init_node:
            rospy.init_node("moveit_client", anonymous=True)

        self.arm_name = rospy.get_param("arm")
        self.arm_base_link_name = 'panda_link0' if self.arm_name=="panda" else "base_footprint" 

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)

    def move_arm(self, pose):
        rospy.wait_for_service('/move_it_planner')
        try:
            c = rospy.ServiceProxy('/move_it_planner', moveToPose)

            # translate to base link reference frame; for some reason its always 0,0,0
            world2Base = self.tfBuffer.lookup_transform('world', self.arm_base_link_name, rospy.Time())
            pose.position.x += world2Base.transform.translation.x
            pose.position.y += world2Base.transform.translation.y
            pose.position.z += world2Base.transform.translation.z

            print(world2Base)

            success = c(pose)
            bool = Bool()
            bool.data = True
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        
        except (rospy.ServiceException, tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Service called failed: %s"%e)

if __name__ == "__main__":
    m = MoveitArmClient(init_node=True)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 0.3
    print("Requesting.. ")
    m.move_arm(pose_goal)

    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.7
    print("Requesting.. ")
    m.move_arm(pose_goal)

    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.4
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.7
    print("Requesting.. ")
    m.move_arm(pose_goal)

    # impossible arm position test
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.3
    # pose_goal.position.y = 0.5
    # pose_goal.position.z = -0.3
    # print("Requesting.. ")
    # m.move_arm(pose_goal)