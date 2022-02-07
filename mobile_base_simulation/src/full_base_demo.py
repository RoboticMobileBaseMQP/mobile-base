from base_package.base_control import BaseControl
from abstract_arm.moveItClient import MoveitArmClient
import time
import geometry_msgs.msg

class FullBaseDemo:
    def __init__(self):
        self.base_controller = BaseControl()
        self.arm_controller = MoveitArmClient()


if __name__ == "__main__":
    f = FullBaseDemo()

    print("moving base to 2,2")
    f.base_controller.base_controllers[0].publish(2)
    f.base_controller.base_controllers[1].publish(2)

    time.sleep(2)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 0.3
    print("moving arm to random position")
    f.arm_controller.move_arm(pose_goal)

    time.sleep(6)

    print("moving base back to 0,0")
    f.base_controller.base_controllers[0].publish(0)
    f.base_controller.base_controllers[1].publish(0)