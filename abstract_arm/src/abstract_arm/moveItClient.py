#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from abstract_arm.srv import moveToPose
from abstract_arm.srv import moveToAngles
from abstract_arm.srv import grip
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from math import pi

class MoveitArmClient:
    def __init__(self, init_node=False) -> None:

        if init_node:
            rospy.init_node("moveit_client", anonymous=True)

        tau = 2.0 * pi
        kortex_home_angles = [0, -tau / 15, 0, tau / 4, 0, tau / 6, 0]   
        panda_home_angles = [0, -tau / 6, 0, -tau / 3, 0, tau / 6, 0] 

        self.arm_name = rospy.get_param("arm/name")
        self.arm_base_link_name = 'panda_link0' if self.arm_name=="panda" else "base_footprint" 
        self.home_joint_angles = panda_home_angles if self.arm_name=="panda" else kortex_home_angles

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)

    def move_arm_EE(self, pose):
        # client side used to move the arm to a desired EE pose

        rospy.wait_for_service('/move_it_EE')
        try:
            c = rospy.ServiceProxy('/move_it_EE', moveToPose)

            # translate to base link reference frame
            world2Base = self.tfBuffer.lookup_transform('world', self.arm_base_link_name, rospy.Time())
            print(world2Base.transform.translation)

            # ensure world to base transform is not 0,0,0
            while world2Base.transform.translation.x == 0.0 and world2Base.transform.translation.y == 0.0:
                world2Base = self.tfBuffer.lookup_transform('world', self.arm_base_link_name, rospy.Time())
                print(world2Base.transform.translation)

            pose.position.x += world2Base.transform.translation.x
            pose.position.y += world2Base.transform.translation.y
            pose.position.z += world2Base.transform.translation.z


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
            return

    def move_arm_angles(self, joints):
        # client side used to move the arm to a desired set of joint angles
        rospy.wait_for_service('/move_it_angles')
        try:
            c = rospy.ServiceProxy('/move_it_angles', moveToAngles)
            success = c(joints)
            bool = Bool()
            bool.data = True
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        
        except Exception as e:
            print("Service called failed as: %s"%e)

    def move_gripper(self, finger):
        # client side used to move the grippers to a desired position
        rospy.wait_for_service('/move_it_gripper')
        try:
            c = rospy.ServiceProxy('/move_it_gripper', grip)
            success = c(finger)
            bool = Bool()
            bool.data = True
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        except Exception as e:
            print("Service called failed as: %s"%e)

    def send_arm_home(self):
        self.move_arm_angles(self.home_joint_angles)

    def is_arm_in_home_pos(self):
        return False #TODO: get_pose(), compare to home joints with tolerance




if __name__ == "__main__":
    m = MoveitArmClient(init_node=True)

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = -0.5
    # pose_goal.position.y = -0.5
    # pose_goal.position.z = 0.3
    # print("Requesting.. ")
    # m.move_arm_EE(pose_goal)

    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = -0.2
    # pose_goal.position.z = 0.7
    # print("Requesting.. ")
    # m.move_arm_EE(pose_goal)

    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = -0.4
    # pose_goal.position.y = -0.2
    # pose_goal.position.z = 0.7
    # print("Requesting.. ")
    # m.move_arm_EE(pose_goal)

    m.is_arm_in_home_pos()

    m.move_arm_angles(m.home_joint_angles)

    # Panda finger
    # finger = 0.0
    # m.move_gripper(finger)






    # impossible arm position test
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.3
    # pose_goal.position.y = 0.5
    # pose_goal.position.z = -0.3
    # print("Requesting.. ")
    # m.move_arm(pose_goal)